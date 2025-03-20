package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVisionIO.PhotonVisionIOInputs;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIOInputs inputs = new PhotonVisionIOInputsAutoLogged();
    private final Drivetrain drivetrain;
    private final PhotonVisionIO io;

    public enum Camera {
        /*
        The coordinate system for the camera to robot transforms is somewhat confusing.
        All lengths are in meters, and angles are in degrees.
        Positions are relative to the center of the robot.
        Positive X means that the camera is towards the front of the robot.
        Positive Y is directed to the left of the robot.
        Positive yaw points to the left, i.e. 90 degrees in yaw is directly pointed left.
        Positive pitch is actually pointed down, which is VERY important to remember.
        We still don't know which way roll is tbh.
        */
        LEFT_POINTING(
                "LEFT_CAMERA", 0.75 / 2.0 - 0.025, -(0.75 / 2.0 - 0.085), +0.205, +0, -13.0, +42.545),
        RIGHT_POINTING(
                "RIGHT_CAMERA", 0.75 / 2.0 - 0.035, -(0.75 / 2.0 - 0.02), +0.205, +0, -13.0, -67.141);

        public final PhotonCameraSim cameraSim;
        public final PhotonPoseEstimator poseEstimator;
        public final Transform3d robotToCamera;

        /** Measured robot center to camera lens center, i.e. robot to cam */
        Camera(
                String name,
                double xMeters,
                double yMeters,
                double zMeters,
                double rollDegrees,
                double pitchDegrees,
                double yawDegrees) {
            this(
                    name,
                    VisionConstants.WIDTH_PX,
                    VisionConstants.HEIGHT_PX,
                    VisionConstants.DIAG_FOV_DEGREES,
                    VisionConstants.AVG_ERR_PX,
                    VisionConstants.STD_DEV_ERR_PX,
                    VisionConstants.FPS,
                    VisionConstants.AVG_LATENCY_MS,
                    VisionConstants.STD_DEV_LATENCY_MS,
                    new Transform3d(
                            xMeters,
                            yMeters,
                            zMeters,
                            new Rotation3d(
                                    Math.toRadians(rollDegrees),
                                    Math.toRadians(pitchDegrees),
                                    Math.toRadians(yawDegrees))));
        }

        Camera(
                String name,
                int width,
                int height,
                double fovDeg,
                double avgErrPx,
                double stdDevErrPx,
                double fps,
                double avgLatencyMs,
                double stdDevLatencyMs,
                Transform3d robotToCamera) {
            this.robotToCamera = robotToCamera;
            SimCameraProperties cameraProp = new SimCameraProperties();
            PhotonCamera camera = new PhotonCamera(name);
            cameraProp.setCalibration(width, height, Rotation2d.fromDegrees(fovDeg));
            cameraProp.setFPS(fps);
            cameraProp.setAvgLatencyMs(avgLatencyMs);
            cameraProp.setLatencyStdDevMs(stdDevLatencyMs);
            cameraProp.setCalibError(avgErrPx, stdDevErrPx);
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            poseEstimator =
                    new PhotonPoseEstimator(
                            VisionConstants.FIELD_LAYOUT,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            robotToCamera);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    public PhotonVision(PhotonVisionIO io, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.io = io;
    }

    public Optional<String> validateVisionPose(EstimatedRobotPose pose, PhotonPipelineResult result) {
        // Reject measurement if average ambiguity is below threshold
        try {
            final double averageAmbiguity =
                    result.getTargets().stream()
                            .mapToDouble(target -> target.getPoseAmbiguity())
                            .average()
                            .orElseThrow();
            if (averageAmbiguity > VisionConstants.ambiguityThreshold) {
                return Optional.of("Ambiguity, " + averageAmbiguity);
            }
        } catch (NoSuchElementException e) {
            System.err.println("PhotonResult with no targets found!!!!! (this should *never* happen)");
        }

        // Reject measurement if estimated 3d pose says robot is off/under the ground
        final double distanceFromGround = Math.abs(pose.estimatedPose.getTranslation().getZ());
        if (distanceFromGround > VisionConstants.zTranslationThreshold) {
            return Optional.of("OffTheGround," + distanceFromGround);
        }

        // discard measurments when a SINGLE tag measurement is too far away
        if (pose.targetsUsed.size() == 1) {
            final double distanceToTag =
                    pose.targetsUsed
                            .get(0)
                            .getAlternateCameraToTarget()
                            .getTranslation()
                            .getDistance(Translation3d.kZero);
            if (distanceToTag > VisionConstants.distanceThreshold) {
                return Optional.of("SingleFar, " + distanceToTag);
            }
        }

        // Reject multi-tag measurement more than X meters away from tags on average
        if (pose.targetsUsed.size() > 1) {
            final double averageDistance =
                    pose.targetsUsed.stream()
                            .mapToDouble(
                                    target ->
                                            target
                                                    .getAlternateCameraToTarget()
                                                    .getTranslation()
                                                    .getDistance(Translation3d.kZero))
                            .average()
                            .getAsDouble(); // this WILL never error because we know there's more than 1 target
            final double factor =
                    1 + (pose.targetsUsed.size() - 2) * VisionConstants.multiTagDistanceFactor;
            final double thresholdDistance = VisionConstants.distanceThreshold * factor;
            if (averageDistance > thresholdDistance) {
                Optional.of(
                        String.format(
                                "MultipleFar: dist=%f.3#, threshold=%f.3#", averageDistance, thresholdDistance));
            }
        }

        // Reject based on gyro angle
        if (pose.targetsUsed.size() == 1) {
            Rotation2d tagRotation = pose.estimatedPose.getRotation().toRotation2d();
            Rotation2d gyroRotation = drivetrain.getGyroRotation();
            final double delta = Math.abs(tagRotation.minus(gyroRotation).getDegrees());
            if (delta > VisionConstants.angleThreshold) {
                return Optional.of("AngleWrong, delta=" + delta);
            }
        }

        // we validated!!! no errors [:

        return Optional.empty();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setReferencePose(drivetrain.getPose());
        io.updateVisionSimPose(drivetrain.getPose());
        for (Camera camera : Camera.values()) {
            final var unread = io.getAllUnreadResults(camera);
            for (final var result : unread) {
                // update the camera's pose estimator
                final var option = io.updatePose(camera, result);
                if (option.isPresent()) {
                    final EstimatedRobotPose pose = option.get();
                    final var validation = validateVisionPose(pose, result);
                    final boolean success = validation.isEmpty();
                    Logger.recordOutput("Vision/ErrorMsg", validation.orElse("Valid"));
                    Logger.recordOutput("Vision/Good Measurement?", success);

                    if (success) {
                        Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
                        if (Constants.currentMode == Constants.Mode.REAL) {
                            drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                        }
                    }
                }
            }
        }
    }

    public boolean areAnyCamerasDisconnected() {
        for (Camera camera : Camera.values()) {
            if (!io.isCameraConnected(inputs, camera)) {
                return true;
            }
        }
        return false;
    }
}
