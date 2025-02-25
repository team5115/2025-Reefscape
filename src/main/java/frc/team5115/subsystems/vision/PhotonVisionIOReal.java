package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOReal implements PhotonVisionIO {
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionIOReal() {
        poseEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.FIELD_LAYOUT,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        VisionConstants.ROBOT_TO_CAM);
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        for (Camera camera : Camera.values()) {
            inputs.isConnected[camera.ordinal()] = camera.cameraSim.getCamera().isConnected();
        }
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
        return Camera.values()[0].cameraSim.getCamera().getAllUnreadResults();
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
    public boolean isCameraConnected(PhotonVisionIOInputs inputs, Camera camera) {
        return inputs.isConnected[camera.ordinal()];
    }

    @Override
    public void updateVisionSimPose(Pose2d pose) {}
}
