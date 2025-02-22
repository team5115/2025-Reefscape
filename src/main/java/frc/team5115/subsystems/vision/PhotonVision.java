package frc.team5115.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class PhotonVision extends SubsystemBase {

    private static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private static final List<AprilTag> reefTags = new ArrayList<AprilTag>();
    private final Drivetrain drivetrain;
    private final PhotonVisionIO io;

    public PhotonVision(PhotonVisionIO io, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.io = io;
        switch (Constants.currentMode) {
            case REAL:
                io = new PhotonVisionIOReal();
                io.addCamera(VisionConstants.cameraName, null);
                break;
            case SIM:
                io = new PhotonVisionIOSim();
                io.addSimCamera(
                        VisionConstants.cameraName, 1080, 720, 100, 0, 0, 30, 15, 50, true, true, true);
                break;
            case REPLAY:
                io = new PhotonVisionIOSim();
                break;
            default:
                io = null;
                break;
        }
    }

    @Override
    public void periodic() {
        io.setReferencePose(drivetrain.getPose());
        final var unread = io.getAllUnreadResults();
        EstimatedRobotPose pose = null;
        for (final var result : unread) {
            final var option = io.updatePose(result);
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

    @Override
    public void simulationPeriodic() {
        io.setReferencePose(drivetrain.getPose());
        final var unread = io.getAllUnreadResults();
        EstimatedRobotPose pose = null;
        for (final var result : unread) {
            final var option = io.updatePose(result);
            if (option.isPresent()) {
                pose = option.get();
                drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            }
        }
    }

    public boolean isCameraConnected(String name) {
        return io.isConnected(name);
    }

    public boolean isAnyCameraConnected() {
        return io.isAnyCameraConnected();
    }

    /** Must call this method at robot start! */
    public static void setupReefTags() {
        for (var tag : fieldLayout.getTags()) {
            if (((tag.ID >= 6 && tag.ID <= 11) || (tag.ID >= 17 && tag.ID <= 22))) {
                reefTags.add(tag);
            }
        }
    }

    /**
     * Get the pose of the nearest april tag on the reef
     *
     * @param robot the pose of the robot
     * @return the pose of the nearest reef april tag
     */
    public static Pose2d getNearestReefTagPose(Pose2d robot) {
        double shortestDistance = Double.MAX_VALUE;
        Pose2d closestPose = null;
        for (var tag : reefTags) {
            final var pose = tag.pose.toPose2d();
            final double distance = pose.getTranslation().getDistance(robot.getTranslation());
            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestPose = pose;
            }
        }
        return closestPose;
    }
}
