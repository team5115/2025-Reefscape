package frc.team5115.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.team5115.Constants.VisionConstants;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOReal implements PhotonVisionIO {
    public Map<String, PhotonCamera> cameras = new HashMap<String, PhotonCamera>();
    private static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionIOReal() {
        poseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.robotToCam);
    }

    @Override
    public boolean isConnected(String name) {
        return cameras.get(name).isConnected();
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
        return cameras.get(VisionConstants.cameraName).getAllUnreadResults();
    }

    @Override
    public Optional<EstimatedRobotPose> updatePose(PhotonPipelineResult result) {
        var pose = poseEstimator.update(result);
        return pose;
    }

    @Override
    public void setReferencePose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
    }

    @Override
    public void addCamera(String name, SimCameraProperties properties) {
        PhotonCamera camera = new PhotonCamera(name);
        cameras.put(name, camera);
    }

    @Override
    public void removeCamera(String name) {
        cameras.remove(name);
    }

    @Override
    public boolean isAnyCameraConnected() {
        return cameras.size() > 0 ? true : false;
    }
}
