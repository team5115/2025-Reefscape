package frc.team5115.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class PhotonVision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final PhotonVisionIO io;

    public PhotonVision(PhotonVisionIO io, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.io = io;
        // ! Note: This switch statement would be better off separated into the constructors of each
        // PhotonVisionIO class
        switch (Constants.currentMode) {
            case REAL:
                io.addCamera(VisionConstants.CAMERA_NAME, null);
                break;
            case SIM:
                io.addSimCamera(
                        VisionConstants.CAMERA_NAME, 1080, 720, 100, 0, 0, 30, 15, 50, true, true, true);
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        io.setReferencePose(drivetrain.getPose());
        final var unread = io.getAllUnreadResults();
        EstimatedRobotPose pose = null;
        for (final var result : unread) {
            final var option = io.updatePose(result);
            if (option.isPresent()) {
                pose = option.get();
                drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
            }
        }
        final boolean hasMeasurement = pose != null;
        Logger.recordOutput("Vision/HasMeasurement", hasMeasurement);
        if (hasMeasurement) {
            Logger.recordOutput("Vision/EstimatedPose", pose.estimatedPose);
        }
    }

    public boolean isCameraConnected(String name) {
        return io.isConnected(name);
    }

    public boolean isAnyCameraConnected() {
        return io.isAnyCameraConnected();
    }
}
