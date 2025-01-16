package frc.team5115.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    // TODO determine max speed, accel, and volts for arm
    private final double maxSpeedDegreesPerSecond = 175.0;
    private final double maxAccelerationDegreesPerSecondPerSecond = 300.0;
    private final double maxVolts = 10.0;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmFeedforward feedforward; // radians
    private final ProfiledPIDController pid; // degrees
    private final SysIdRoutine sysId;
    private State state = State.INITIAL;

    public enum State {
        INITIAL(+75.0),
        STOWED(+75.0),
        INTAKE(+0.0),
        CLOSE_SHOT(+12.0),
        MEDIUM_SHOT(+15.0),
        FAR_SHOT(+25.0),
        AMP(+98.0);

        private final double position;

        State(double degrees) {
            position = degrees;
        }
    }

    public Arm(ArmIO io) {
        this.io = io;
        final var constraints =
                new TrapezoidProfile.Constraints(
                        maxSpeedDegreesPerSecond, maxAccelerationDegreesPerSecondPerSecond);
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                // TODO tune arm feedforward and pid
                feedforward = new ArmFeedforward(0.97072, 0.62983, 0.838, 0.94448);
                pid = new ProfiledPIDController(0.072, 0.0, 0.0, constraints);
                break;
            case SIM:
                feedforward = new ArmFeedforward(0.0, 0.35, 0.135, 0.05);
                pid = new ProfiledPIDController(0.3, 0.0, 0.0, constraints);
                break;
            default:
                feedforward = new ArmFeedforward(0.0, 0.0, 0, 0.0);
                pid = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
                break;
        }

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setArmVoltage(voltage.magnitude()), null, this));

        pid.setTolerance(3.0);
        pid.setGoal(91.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/Goal Degrees", pid.getGoal().position);
        Logger.recordOutput("Arm/Actual Degrees", inputs.armAngle.getDegrees());
        Logger.recordOutput("Arm/Velocity RadsPerSec", inputs.armVelocityRPM * 2 * Math.PI / 60.0);
        Logger.recordOutput("Arm/Position Rads", inputs.armAngle.getRadians());
        Logger.recordOutput("Arm/At Goal?", pid.atGoal());
        Logger.recordOutput("Arm/State", getStateString());
        Logger.recordOutput(
                "Arm/Offset Delta", Math.abs(pid.getGoal().position - inputs.armAngle.getDegrees()));

        // The feedforward uses the pid's trapezoidal setpoint to counteract gravity and kstatic
        io.setArmVoltage(
                MathUtil.clamp(
                        pid.calculate(inputs.armAngle.getDegrees())
                                + feedforward.calculate(
                                        Math.toRadians(pid.getSetpoint().position),
                                        Math.toRadians(pid.getSetpoint().velocity)),
                        -maxVolts,
                        +maxVolts));
    }

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> pid.atGoal()).withTimeout(timeout);
    }

    public Command setState(State state) {
        return Commands.runOnce(
                () -> {
                    this.state = state;
                    pid.setGoal(state.position);
                });
    }

    public Command setStateAndWait(State state, double timeoutSeconds) {
        return setState(state).andThen(waitForSetpoint(timeoutSeconds));
    }

    public Command stow() {
        return setState(State.STOWED);
    }

    public Command intake() {
        return setState(State.INTAKE);
    }

    public Command stowAndWait(double timeoutSeconds) {
        return setStateAndWait(State.STOWED, timeoutSeconds);
    }

    public Command intakeAndWait(double timeoutSeconds) {
        return setStateAndWait(State.INTAKE, timeoutSeconds);
    }

    private String getStateString() {
        if (pid.atGoal()) {
            return state.toString();
        } else {
            return "MOVING_TO_" + state.toString();
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    // manipulate arm with joystick
    @Deprecated
    public void goAtVoltage(double speed) {
        io.setArmVoltage(speed * maxVolts);
    }
}
