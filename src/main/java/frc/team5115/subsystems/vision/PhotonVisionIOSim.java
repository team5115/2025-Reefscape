package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionIOSim implements PhotonVisionIO {
    private final VisionSystemSim visionSim;

    public PhotonVisionIOSim() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        for (Camera camera : Camera.values()) {
            camera.cameraSim.enableRawStream(true);
            camera.cameraSim.enableProcessedStream(true);
            camera.cameraSim.enableDrawWireframe(true);
            visionSim.addCamera(camera.cameraSim, VisionConstants.ROBOT_TO_CAM);
        }
    }

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
    public void updateVisionSimPose(Pose2d pose) {
        visionSim.update(pose);
    }

    //     @Override
    //     public void addSimCamera(
    //             String name,
    //             int width,
    //             int height,
    //             int fovDeg,
    //             int avgErrPx,
    //             int stdDevErrPx,
    //             int fps,
    //             int avgLatencyMs,
    //             int stdDevLatencyMs,
    //             boolean rawStreamEnabled,
    //             boolean processedStreamEnabled,
    //             boolean wireframeEnabled) {

    //         PhotonCamera camera = new PhotonCamera(name);
    //         SimCameraProperties cameraProp = new SimCameraProperties();
    //         cameraProp.setCalibration(width, height, Rotation2d.fromDegrees(fovDeg));
    //         cameraProp.setCalibError(avgErrPx, stdDevErrPx);
    //         cameraProp.setFPS(fps);
    //         cameraProp.setAvgLatencyMs(avgLatencyMs);
    //         cameraProp.setLatencyStdDevMs(stdDevLatencyMs);

    //         PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
    //         visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAM);

    //         cameras.put(name, cameraSim);
    //         cameraSim.enableDrawWireframe(wireframeEnabled);
    //         cameraSim.enableProcessedStream(processedStreamEnabled);
    //         cameraSim.enableRawStream(rawStreamEnabled);
    //     }
}
