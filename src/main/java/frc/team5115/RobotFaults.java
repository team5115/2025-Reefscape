package frc.team5115;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkMax;
import frc.team5115.subsystems.climber.Climber;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.elevator.Elevator;
import frc.team5115.subsystems.intake.Intake;
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
    public final boolean climberNull;
    public final boolean elevatorNull;
    public final boolean dispenserNull;
    public final boolean intakeNull;
    public final boolean elevatorShorted;
    private final String cachedToString;

    public RobotFaults(
            String sparkFaults,
            boolean cameraDisconnected,
            boolean joysticksDisconnected,
            boolean gyroDisconnected,
            boolean drivetrainNull,
            boolean visionNull,
            boolean climberNull,
            boolean elevatorNull,
            boolean dispenserNull,
            boolean intakeNull,
            boolean elevatorShorted) {
        this.sparkFaults = sparkFaults;
        this.cameraDisconnected = cameraDisconnected;
        this.joysticksDisconnected = joysticksDisconnected;
        this.gyroDisconnected = gyroDisconnected;
        this.drivetrainNull = drivetrainNull;
        this.visionNull = visionNull;
        this.climberNull = climberNull;
        this.elevatorNull = elevatorNull;
        this.dispenserNull = dispenserNull;
        this.intakeNull = intakeNull;
        this.elevatorShorted = elevatorShorted;
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
        if (climberNull) {
            builder.append("ClimberNull; ");
        }
        if (elevatorNull) {
            builder.append("ElevatorNull; ");
        }
        if (dispenserNull) {
            builder.append("DispenserNull; ");
        }
        if (intakeNull) {
            builder.append("IntakeNull; ");
        }
        if (elevatorShorted) {
            builder.append("5V Short; ");
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
            Drivetrain drivetrain,
            PhotonVision vision,
            Climber climber,
            Elevator elevator,
            Dispenser dispenser,
            Intake intake,
            Dealgaefacationinator5000 dealgaefacationinator5000,
            boolean joysticksConnected) {

        ArrayList<SparkMax> sparks = new ArrayList<>();
        if (drivetrain != null) {
            drivetrain.getSparks(sparks);
        }
        if (elevator != null) {
            elevator.getSparks(sparks);
        }
        if (dispenser != null) {
            dispenser.getSparks(sparks);
        }
        if (intake != null) {
            intake.getSparks(sparks);
        }
        if (dealgaefacationinator5000 != null) {
            dealgaefacationinator5000.getSparks(sparks);
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
                vision == null,
                climber == null,
                elevator == null,
                dispenser == null,
                intake == null,
                elevator.isShorting());
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
