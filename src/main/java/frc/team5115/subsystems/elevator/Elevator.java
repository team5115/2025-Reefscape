package frc.team5115.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    // TODO determine max speed, accel, and volts for elevator
    private final double maxSpeedMetersPerSecond = 4.0;
    private final double maxAccelerationMetersPerSecondPerSecond = 12.0;
    private final double maxVolts = 10.0;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ElevatorFeedforward feedforward; // meters
    private final ProfiledPIDController pid; // meters
    private final SysIdRoutine sysId;
    private Height height = Height.BOTTOM;

    public enum Height {
        BOTTOM(0),
        MIDDLE(0.6),
        TOP(1.5);

        public final double position;

        Height(double meters) {
            position = meters;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
        final var constraints =
                new TrapezoidProfile.Constraints(
                        maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondPerSecond);
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                // TODO tune elevator feedforward and pid
                feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
                pid = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
                break;
            case SIM:
                feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
                pid = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
                break;
            default:
                feedforward = new ElevatorFeedforward(0.0, 0.0, 0, 0.0);
                pid = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
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

        pid.setTolerance(0.05);
        height = Height.BOTTOM;
        pid.setGoal(height.position);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        recordOutputs();
        io.setElevatorVoltage(
                MathUtil.clamp(
                        pid.calculate(inputs.positionMeters)
                                + feedforward.calculate(pid.getSetpoint().velocity),
                        -maxVolts,
                        maxVolts));
    }

    private void recordOutputs() {
        Logger.recordOutput("Elevator/Goal Height", pid.getGoal().position);
        Logger.recordOutput("Elevator/Setpoint Height", pid.getSetpoint().position);
        Logger.recordOutput("Elevator/Actual Height", inputs.positionMeters);

        Logger.recordOutput("Elevator/Goal Velocity", pid.getGoal().velocity);
        Logger.recordOutput("Elevator/Setpoint Velocity", pid.getSetpoint().velocity);
        Logger.recordOutput("Elevator/Actual Velocity", inputs.velocityMetersPerSecond);

        Logger.recordOutput("Elevator/At Goal?", pid.atGoal());
        Logger.recordOutput("Elevator/State", getStateString());
        Logger.recordOutput("Elevator/Offset Delta", pid.getGoal().position - inputs.positionMeters);
    }

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> pid.atGoal()).withTimeout(timeout);
    }

    public Command setHeight(Height height) {
        return Commands.runOnce(
                () -> {
                    this.height = height;
                    pid.setGoal(height.position);
                });
    }

    public Command setHeightAndWait(Height height, double timeoutSeconds) {
        return setHeight(height).andThen(waitForSetpoint(timeoutSeconds));
    }

    private String getStateString() {
        if (pid.atGoal()) {
            return height.toString();
        } else {
            return "MOVING_TO_" + height.toString();
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    // Manipulate elevator by percent of max volts
    // Comment out the call to `io.setElevatorVoltage()` in `periodic()` in order for this to work
    @Deprecated
    public void manualControl(double percent) {
        io.setElevatorVoltage(percent * maxVolts);
    }
}
