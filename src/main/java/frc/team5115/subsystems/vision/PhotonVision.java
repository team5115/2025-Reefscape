package frc.team5115.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVision(final Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        poseEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.FIELD_LAYOUT,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        VisionConstants.ROBOT_TO_CAM);
    }

    @Override
    public void periodic() {
        poseEstimator.setReferencePose(drivetrain.getPose());
        final var unread = camera.getAllUnreadResults();
        EstimatedRobotPose pose = null;
        for (final var result : unread) {
            final var option = poseEstimator.update(result);
            if (option.isPresent()) {
                pose = option.get();
                drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
            }
        }
        Logger.recordOutput("Vision/HasMeasurement", pose != null);
    }

    public boolean isCameraConnected() {
        return camera.isConnected();
    }
}
