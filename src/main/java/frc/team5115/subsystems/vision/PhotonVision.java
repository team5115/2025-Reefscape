package frc.team5115.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVisionIO.PhotonVisionIOInputs;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIOInputs inputs = new PhotonVisionIOInputsAutoLogged();
    private final Drivetrain drivetrain;
    private final PhotonVisionIO io;

    public enum Camera {
        USB_GS_Camera("USB_GS_Camera", 1080, 720, 90, 0, 0, 30, 15, 50);

        public PhotonCameraSim cameraSim;

        Camera(
                String name,
                int width,
                int height,
                int fovDeg,
                int avgErrPx,
                int stdDevErrPx,
                int fps,
                int avgLatencyMs,
                int stdDevLatencyMs) {
            SimCameraProperties cameraProp = new SimCameraProperties();
            PhotonCamera camera = new PhotonCamera(name);
            cameraProp.setCalibration(width, height, Rotation2d.fromDegrees(fovDeg));
            cameraProp.setFPS(fps);
            cameraProp.setAvgLatencyMs(avgLatencyMs);
            cameraProp.setLatencyStdDevMs(stdDevLatencyMs);
            cameraProp.setCalibError(avgErrPx, stdDevErrPx);
            cameraSim = new PhotonCameraSim(camera, cameraProp);
        }
    }

    public PhotonVision(PhotonVisionIO io, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.io = io;
        // ! Note: This switch statement would be better off separated into the constructors of each
        // PhotonVisionIO class
        switch (Constants.currentMode) {
            case REAL:
                break;
            case SIM:
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
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

    public boolean isAnyCameraConnected() {
        for (Camera camera : Camera.values()) {
            if (io.isCameraConnected(inputs, camera)) {
                return true;
            }
        }
        return false;
    }
}
