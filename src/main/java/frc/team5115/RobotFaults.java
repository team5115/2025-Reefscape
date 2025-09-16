package frc.team5115;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkMax;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVision;
import java.util.ArrayList;

public class RobotFaults {
    private static final String NO_FAULTS = "No Faults";
    public final String sparkFaults;
    public final boolean cameraDisconnected;
    public final boolean joysticksDisconnected;
    public final boolean gyroDisconnected;
    public final boolean drivetrainNull;
    public final boolean visionNull;
    private final String cachedToString;

    public RobotFaults(
            String sparkFaults,
            boolean cameraDisconnected,
            boolean joysticksDisconnected,
            boolean gyroDisconnected,
            boolean drivetrainNull,
            boolean visionNull) {
        this.sparkFaults = sparkFaults;
        this.cameraDisconnected = cameraDisconnected;
        this.joysticksDisconnected = joysticksDisconnected;
        this.gyroDisconnected = gyroDisconnected;
        this.drivetrainNull = drivetrainNull;
        this.visionNull = visionNull;
        cachedToString = cacheString();
    }

    public String cacheString() {
        final StringBuilder builder = new StringBuilder();
        if (!sparkFaults.isEmpty()) {
            builder.append("SparkFaults:[ ");
            builder.append(sparkFaults);
            builder.append("] ; ");
        }
        if (cameraDisconnected) {
            builder.append("CameraDisconnected; ");
        }
        if (joysticksDisconnected) {
            builder.append("JoysticksDisconnected; ");
        }
        if (gyroDisconnected) {
            builder.append("GyroDisconnected; ");
        }
        if (drivetrainNull) {
            builder.append("DrivetrainNull; ");
        }
        if (visionNull) {
            builder.append("VisionNull; ");
        }
        if (builder.isEmpty()) {
            return NO_FAULTS;
        } else {
            return "HAS FAULTS! " + builder.toString();
        }
    }

    @Override
    public String toString() {
        return cachedToString;
    }

    public boolean hasFaults() {
        return !cachedToString.equals(NO_FAULTS);
    }

    public static RobotFaults fromSubsystems(
            Drivetrain drivetrain, PhotonVision vision, boolean joysticksConnected) {

        ArrayList<SparkMax> sparks = new ArrayList<>();
        if (drivetrain != null) {
            drivetrain.getSparks(sparks);
        }
        StringBuilder sparkFaults = new StringBuilder();
        for (var spark : sparks) {
            appendSparkFaults(sparkFaults, spark.getFaults(), spark.getDeviceId());
        }
        return new RobotFaults(
                sparkFaults.toString(),
                vision == null ? true : vision.areAnyCamerasDisconnected(),
                !joysticksConnected,
                drivetrain == null ? true : !drivetrain.isGyroConnected(),
                drivetrain == null,
                vision == null);
    }

    private static void appendSparkFaults(StringBuilder mainBuilder, Faults faults, int id) {
        final StringBuilder builder = new StringBuilder();
        if (faults.other) {
            builder.append("OtherFault,");
        }
        if (faults.motorType) {
            builder.append("MotorTypeFault,");
        }
        if (faults.sensor) {
            builder.append("SensorFault,");
        }
        if (faults.can) {
            builder.append("CanFault,");
        }
        if (faults.temperature) {
            builder.append("TemperatureFault,");
        }
        if (faults.gateDriver) {
            builder.append("GateDriverFault,");
        }
        if (faults.escEeprom) {
            builder.append("EscEepromFault,");
        }
        if (faults.firmware) {
            builder.append("FirmwareFault,");
        }
        if (!builder.isEmpty()) {
            // SparkFaults:[ { ID_0,GateDriverFault },{ ID_1,FirmwareFault}, ] ;
            mainBuilder.append("{ ID_");
            mainBuilder.append(id);
            mainBuilder.append(",");
            mainBuilder.append(builder);
            mainBuilder.append(" },");
        }
    }
}
