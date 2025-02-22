package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {}

    public default boolean isConnected(String name) {
        return false;
    }

    public default List<PhotonPipelineResult> getAllUnreadResults() {
        return null;
    }

    public default Optional<EstimatedRobotPose> updatePose(PhotonPipelineResult result) {
        return null;
    }

    public default void setReferencePose(Pose2d pose) {}

    public default void addCamera(String name, SimCameraProperties cameraProp) {}

    public default void addSimCamera(
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
            boolean wireframeEnabled) {}

    public default void removeCamera(String name) {}

    public default boolean isAnyCameraConnected() {
        return true;
    }
}
