package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVisionIO.PhotonVisionIOInputs;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIOInputs inputs = new PhotonVisionIOInputsAutoLogged();
    private final Drivetrain drivetrain;
    private final PhotonVisionIO io;

    public enum Camera {
        FRONT_LEFT("LEFT_CAMERA", 0.36, -0.30, 0.20828, 0, 13.0, 36.085),
        ; // FRONT_RIGHT("RIGHT_CAMERA", 0.385, -0.37 - 0.2 - 0.07 - 0.02, 0.21, 0, 13.0, 41.5 - 116 +
        // 7);

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
        }
    }

    public PhotonVision(PhotonVisionIO io, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setReferencePose(drivetrain.getPose());
        io.updateVisionSimPose(drivetrain.getPose());
        for (Camera camera : Camera.values()) {
            final var unread = io.getAllUnreadResults(camera);
            EstimatedRobotPose pose = null;
            for (final var result : unread) {
                final var option = io.updatePose(camera, result);
                if (option.isPresent()) {
                    pose = option.get();
                    drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                    Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
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
