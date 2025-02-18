package frc.team5115.subsystems.vision;

import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {
    }

    public default boolean isConnected() { return false; }
    
    public default List<PhotonPipelineResult> getAllUnreadResults() { return null; }

    public default EstimatedRobotPose updatePose() { return null; }

    public default void setReferencePose() {}

    public default void setPoseEstimator() {}
}
