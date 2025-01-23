package frc.team5115.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        camera = new PhotonCamera(VisionConstants.cameraName);
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        poseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.robotToCam);
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
            }
        }

        final boolean hasMeasurement = pose != null;
        Logger.recordOutput("Vision/HasMeasurement", hasMeasurement);
        if (hasMeasurement) {
            Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
        }
    }

    public Pose2d getPoseRelative() {
        var results = camera.getAllUnreadResults();
        Pose2d pose = null;
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if ((target.getFiducialId() > 6 && target.getFiducialId() < 11)
                            || (target.getFiducialId() > 17 && target.getFiducialId() < 22)) {
                        Transform3d transform =
                                target.getBestCameraToTarget().plus(VisionConstants.robotToCam.inverse());
                        pose =
                                new Pose2d(
                                        transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
                    }
                }
            }
        }
        return pose;
    }
}
