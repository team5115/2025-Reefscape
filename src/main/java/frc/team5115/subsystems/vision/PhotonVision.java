package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
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
        USB_GS_Camera("USB_GS_Camera", 1280, 720, 90, 1, 0, 30, 30, 10, VisionConstants.ROBOT_TO_CAM);
        // USB_GS_Camera2("USB_GS_Camera2", 640, 360, 360, 1, 1, 30, 30, 10, VisionConstants.ROBOT_TO_CAM);

        public final PhotonCameraSim cameraSim;
        public final PhotonPoseEstimator poseEstimator;

        Camera(
                String name,
                int width,
                int height,
                int fovDeg,
                int avgErrPx,
                int stdDevErrPx,
                int fps,
                int avgLatencyMs,
                int stdDevLatencyMs,
                Transform3d robotToCamera) {
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

    public boolean isAnyCameraConnected() {
        for (Camera camera : Camera.values()) {
            if (io.isCameraConnected(inputs, camera)) {
                return true;
            }
        }
        return false;
    }
}
