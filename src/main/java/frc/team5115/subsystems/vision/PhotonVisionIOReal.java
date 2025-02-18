package frc.team5115.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.PoseEstimator;
import frc.team5115.Constants.VisionConstants;

public class PhotonVisionIOReal implements PhotonVisionIO {
    private PhotonCamera camera;
    private final PoseEstimator poseEstimator;

    public PhotonVisionIOReal(){
        camera = new PhotonCamera(VisionConstants.cameraName);
        poseEstimator = null;
    }
}
