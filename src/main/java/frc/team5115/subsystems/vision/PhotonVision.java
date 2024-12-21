package frc.team5115.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;

public class PhotonVision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        camera = new PhotonCamera(VisionConstants.cameraName);
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.robotToCam);
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
            }
        }

        final boolean hasMeasurement = pose != null;
        Logger.recordOutput("Vision/HasMeasurement", hasMeasurement);
        if (hasMeasurement) {
            drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
        }
    }
}
