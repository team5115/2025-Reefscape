package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOReal implements PhotonVisionIO {
    public PhotonVisionIOReal() {}

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        for (Camera camera : Camera.values()) {
            inputs.isConnected[camera.ordinal()] = camera.cameraSim.getCamera().isConnected();
        }
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults(Camera camera) {
        return camera.cameraSim.getCamera().getAllUnreadResults();
    }

    @Override
    public Optional<EstimatedRobotPose> updatePose(Camera camera, PhotonPipelineResult result) {
        var pose = camera.poseEstimator.update(result);
        return pose;
    }

    @Override
    public boolean isCameraConnected(PhotonVisionIOInputs inputs, Camera camera) {
        return inputs.isConnected[camera.ordinal()];
    }

    @Override
    public void updateVisionSimPose(Pose2d pose) {}
}
