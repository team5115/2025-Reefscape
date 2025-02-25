package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team5115.subsystems.vision.PhotonVision.Camera;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
    // TODO add some inputs, e.g. a boolean for each camera for if that cam is connected
    @AutoLog
    public static class PhotonVisionIOInputs {
        public boolean[] isConnected = new boolean[Camera.values().length];
    }

    public default void updateInputs(PhotonVisionIOInputs inputs) {}

    // TODO: add an updateInputs function and override it in implementations

    public default List<PhotonPipelineResult> getAllUnreadResults() {
        return null;
    }

    public default Optional<EstimatedRobotPose> updatePose(PhotonPipelineResult result) {
        return null;
    }

    public default void updateVisionSimPose(Pose2d pose) {}

    public default void setReferencePose(Pose2d pose) {}

    public default boolean isCameraConnected(PhotonVisionIOInputs inputs, Camera camera) {
        return false;
    }
}
