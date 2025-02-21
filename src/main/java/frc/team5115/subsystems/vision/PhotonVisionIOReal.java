package frc.team5115.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.team5115.Constants.VisionConstants;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOReal implements PhotonVisionIO {
    private PhotonCamera camera;
    private static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionIOReal() {
        camera = new PhotonCamera(VisionConstants.cameraName);
        poseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.robotToCam);
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
        return camera.getAllUnreadResults();
    }

    @Override
    public EstimatedRobotPose updatePose(PhotonPipelineResult result) {
        var pose = poseEstimator.update(result);
        return pose.get();
    }

    @Override
    public void setReferencePose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
    }
}
