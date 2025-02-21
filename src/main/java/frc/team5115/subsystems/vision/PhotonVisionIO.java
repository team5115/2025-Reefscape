package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {}

    public default boolean isConnected() {
        return false;
    }

    public default List<PhotonPipelineResult> getAllUnreadResults() {
        return null;
    }

    public default EstimatedRobotPose updatePose(PhotonPipelineResult result) {
        return null;
    }

    public default void setReferencePose(Pose2d pose) {}
}
