package frc.team5115.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
    // TODO determine max speed, max volts, kG for elevator
    private static final double maxSpeed = 4.0; // m/s
    private static final double maxVolts = 10.0;
    private static final double kgVolts = 0.9;
    private static final double minHeightInches = 30; // TODO: find minimum height
     // TODO find sensor heights
    private static final double firstHeight = 0;
    private static final double secondHeight = 0;
    private static final double thirdHeight = 0;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController positionPID; // control meters, output m/s
    private final SysIdRoutine sysId;
    private Height height = Height.L2;
    private double velocitySetpoint;
    public double offset;

    @AutoLogOutput
    private final LoggedMechanism2d elevatorMechanism2d =
            new LoggedMechanism2d(10, Height.L4.position * 10);

    private final LoggedMechanismRoot2d elevatorMechanismRoot2d =
            elevatorMechanism2d.getRoot(getName() + " Root", 0, 0);
    private final LoggedMechanismLigament2d elevatorMechanismLigament2d =
            elevatorMechanismRoot2d.append(
                    new LoggedMechanismLigament2d(getName(), 0, 90));
    private final LoggedMechanismLigament2d elevatorMechanismLigament2d2 =
            elevatorMechanismLigament2d.append(new LoggedMechanismLigament2d(getName() + "2", 10, -90));

    public enum Height {
        MINIMUM(minHeightInches),
        L2(30),
        INTAKE(30), // TODO: find intake height
        L3(45),
        L4(71);

        public final double position; // meters

        Height(double inches) {
            position = (inches - minHeightInches) * 0.0254;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
        SmartDashboard.putData("Elevator Mechanism", elevatorMechanism2d);
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                // TODO tune elevator feedforward and pid
                positionPID = new PIDController(0.0, 0.0, 0.0);
                break;
            case SIM:
                positionPID = new PIDController(1.0, 0.0, 0.0);
                break;
            default:
                positionPID = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setElevatorVoltage(voltage.magnitude()), null, this));

        height = Height.L2;
        positionPID.setTolerance(0.05); // meters
        positionPID.setSetpoint(height.position);
    }

    public double getActualHeight() {
        return inputs.positionMeters - offset;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        recordOutputs();
        if (inputs.firstMagnetDetected == true) {
            offset = firstHeight - inputs.positionMeters;
        }
        if (inputs.secondMagnetDetected == true) {
            offset = secondHeight - inputs.positionMeters;
        }
        if (inputs.thirdMagnetDetected == true) {
            offset = thirdHeight - inputs.positionMeters;
        }
        if (inputs.backCoralDetected) {
            // Force the elvator to stay at the intake position when there is a coral in the intake
            height = Height.INTAKE;
        }
        io.setElevatorVelocity(velocitySetpoint, kgVolts);
        elevatorMechanismLigament2d.setLength(getActualHeight() * 8);
    }

    private void recordOutputs() {
        Logger.recordOutput("Elevator/Goal Height", height.position);
        Logger.recordOutput("Elevator/Setpoint Velocity", velocitySetpoint);
        Logger.recordOutput("Elevator/Actual Height", getActualHeight());
        Logger.recordOutput("Elevator/Actual Velocity", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/At Goal?", atGoal());
        Logger.recordOutput("Elevator/State", getStateString());
        Logger.recordOutput("Elevator/Offset Delta", positionPID.getSetpoint() - getActualHeight());
    }

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> atGoal()).withTimeout(timeout);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.backCoralDetected == state).withTimeout(timeout);
    }

    public boolean checkElevator() {
        return atGoal() && positionPID.getSetpoint() == Height.INTAKE.position;
    }

    public Command setHeight(Height height) {
        return Commands.runOnce(
                () -> {
                    this.height = height;
                    positionPID.setSetpoint(height.position);
                });
    }

    public Command setHeightAndWait(Height height, double timeoutSeconds) {
        return setHeight(height).andThen(waitForSetpoint(timeoutSeconds));
    }

    private String getStateString() {
        if (atGoal()) {
            return height.toString();
        } else {
            return "MOVING_TO_" + height.toString();
        }
    }

    public boolean atGoal() {
        return Math.abs(velocitySetpoint - inputs.velocityMetersPerSecond) <= 0.1
                && positionPID.atSetpoint();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    public Command velocityControl(DoubleSupplier speedMetersPerSecond) {
        return Commands.runOnce(() -> velocitySetpoint = speedMetersPerSecond.getAsDouble(), this);
    }

    public Command positionControl() {
        return Commands.run(
                () -> {
                    velocitySetpoint =
                            MathUtil.clamp(
                                    positionPID.calculate(getActualHeight(), height.position),
                                    -maxSpeed,
                                    +maxSpeed);
                },
                this);
    }
}
