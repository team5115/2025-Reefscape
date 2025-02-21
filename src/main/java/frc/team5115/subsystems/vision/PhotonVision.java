package frc.team5115.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonVision extends SubsystemBase {

    private static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private static final List<AprilTag> reefTags = new ArrayList<AprilTag>();
    private final Drivetrain drivetrain;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;

    public PhotonVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        camera = new PhotonCamera(VisionConstants.cameraName);
        poseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.robotToCam);
        switch (Constants.currentMode) {
            case REAL:
                visionSim = null;
                cameraSim = null;
                break;
            case SIM:
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(fieldLayout);
                var cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(1080, 920, Rotation2d.fromDegrees(0));
                cameraProp.setCalibError(0, 0);
                cameraProp.setFPS(30);
                cameraProp.setAvgLatencyMs(50);
                cameraProp.setLatencyStdDevMs(15);
                cameraSim = new PhotonCameraSim(camera, cameraProp);
                visionSim.addCamera(cameraSim, VisionConstants.robotToCam);
                cameraSim.enableDrawWireframe(true);
                break;
            case REPLAY:
                visionSim = null;
                cameraSim = null;
                break;
            default:
                visionSim = null;
                cameraSim = null;
                break;
        }
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

    public boolean isCameraConnected() {
        return camera.isConnected();
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
