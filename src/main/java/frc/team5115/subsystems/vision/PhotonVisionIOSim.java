package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants.VisionConstants;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOSim implements PhotonVisionIO {
    public final Map<String, PhotonCameraSim> cameras = new HashMap<String, PhotonCameraSim>();
    private final PhotonPoseEstimator poseEstimator;
    private final VisionSystemSim visionSim;

    public PhotonVisionIOSim() {
        poseEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.FIELD_LAYOUT,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        VisionConstants.ROBOT_TO_CAM);
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
    }

    @Override
    public boolean isConnected(String name) {
        return cameras.get(name).getCamera().isConnected();
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
        return cameras.get(VisionConstants.CAMERA_NAME).getCamera().getAllUnreadResults();
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
    public void addSimCamera(
            String name,
            int width,
            int height,
            int fovDeg,
            int avgErrPx,
            int stdDevErrPx,
            int fps,
            int avgLatencyMs,
            int stdDevLatencyMs,
            boolean rawStreamEnabled,
            boolean processedStreamEnabled,
            boolean wireframeEnabled) {

        PhotonCamera camera = new PhotonCamera(name);
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(width, height, Rotation2d.fromDegrees(fovDeg));
        cameraProp.setCalibError(avgErrPx, stdDevErrPx);
        cameraProp.setFPS(fps);
        cameraProp.setAvgLatencyMs(avgLatencyMs);
        cameraProp.setLatencyStdDevMs(stdDevLatencyMs);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAM);

        cameras.put(name, cameraSim);
        cameraSim.enableDrawWireframe(wireframeEnabled);
        cameraSim.enableProcessedStream(processedStreamEnabled);
        cameraSim.enableRawStream(rawStreamEnabled);
    }

    @Override
    public void removeCamera(String name) {
        visionSim.removeCamera(cameras.get(name));
    }

    @Override
    public boolean isAnyCameraConnected() {
        return cameras.size() > 0 ? true : false;
    }
}
