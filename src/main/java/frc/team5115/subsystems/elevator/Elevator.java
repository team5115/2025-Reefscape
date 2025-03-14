package frc.team5115.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
    private static final double maxAcceleration = 20.0;
    private static final double maxSpeed = 10.0; // m/s
    private static final double minHeightInches = 22.25;
    private static final double firstHeight = 0;
    // private static final double secondHeight = 0;
    // private static final double thirdHeight = 0;
    // private static final double fourthHeight = 0;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController positionPID; // control meters, output m/s
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
            elevatorMechanismRoot2d.append(new LoggedMechanismLigament2d(getName(), 0, 90));
    private final LoggedMechanismLigament2d elevatorMechanismLigament2d2 =
            elevatorMechanismLigament2d.append(new LoggedMechanismLigament2d(getName() + "2", 10, -90));

    public enum Height {
        MINIMUM(minHeightInches),
        INTAKE(minHeightInches),
        L1(minHeightInches + 7.0),
        L2(minHeightInches + 14.85),
        CLEAN2(minHeightInches + 14.85 + 6),
        L3(52),
        CLEAN3(52 + 6),
        L4(60);

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
                positionPID =
                        new ProfiledPIDController(
                                1.6, 0.0, 0.0, new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration));
                break;
            case SIM:
                positionPID =
                        new ProfiledPIDController(
                                1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration));
                break;
            default:
                positionPID =
                        new ProfiledPIDController(
                                0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration));
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

        height = Height.MINIMUM;
        positionPID.setTolerance(0.05);
        positionPID.setGoal(height.position);
    }

    public double getActualHeight() {
        return inputs.positionMeters + offset;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        recordOutputs();
        if (inputs.magnet1detected) {
            offset = firstHeight - inputs.positionMeters;
        }
        // if (inputs.magnet2detected) {
        //     offset = secondHeight - inputs.positionMeters;
        // }
        // if (inputs.magnet3detected) {
        //     offset = thirdHeight - inputs.positionMeters;
        // }
        // if (inputs.magnet4detected) {
        //     offset = fourthHeight - inputs.positionMeters;
        // }
        // if (inputs.backCoralDetected) {
        //     // Force the elvator to stay at the intake position when there is a coral in the intake
        //     height = Height.INTAKE;
        // }
        io.setElevatorVelocity(velocitySetpoint, 0);
        elevatorMechanismLigament2d.setLength(getActualHeight() * 8);
    }

    /**
     * Move the elevator down until it reaches the bottom sensor, then zero offset
     *
     * @return a command that does so
     */
    public Command zero() {
        final var retval =
                Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    velocitySetpoint = -0.10;
                                }),
                        Commands.waitUntil(() -> inputs.magnet1detected == true).withTimeout(3.0),
                        Commands.runOnce(
                                () -> {
                                    velocitySetpoint = 0;
                                    offset = -inputs.positionMeters;
                                }));
        retval.addRequirements(this);
        return retval;
    }

    private void recordOutputs() {
        Logger.recordOutput("Elevator/Goal Height", height.position);
        Logger.recordOutput("Elevator/Setpoint Velocity", velocitySetpoint);
        Logger.recordOutput("Elevator/Actual Height", getActualHeight());
        Logger.recordOutput(
                "Elevator/Inches From Ground", getActualHeight() * 100d / 2.54 + minHeightInches);
        Logger.recordOutput("Elevator/Actual Velocity", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/At Goal?", atGoal());
        Logger.recordOutput("Elevator/State", getStateString());
        Logger.recordOutput(
                "Elevator/Offset Delta", positionPID.getGoal().position - getActualHeight());
        Logger.recordOutput("Elevator/OffsetValue", offset);
    }

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> atGoal()).withTimeout(timeout);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.backCoralDetected == state).withTimeout(timeout);
    }

    public boolean atIntake() {
        return atGoal() && positionPID.getGoal().position == Height.INTAKE.position;
    }

    public Command setHeight(Height height) {
        return Commands.runOnce(
                () -> {
                    this.height = height;
                    positionPID.setGoal(height.position);
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
        return Commands.run(() -> velocitySetpoint = speedMetersPerSecond.getAsDouble(), this);
    }

    public Command positionControl() {
        return Commands.run(
                () -> {
                    velocitySetpoint = positionPID.calculate(getActualHeight(), height.position);
                },
                this);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }

    public double getDispenserSpeed() {
        if (getActualHeight() <= (Height.L2.position + Height.L1.position) / 2) {
            return 0.45; // TODO find L1 height
        } else {
            return 0.70;
        }
    }
}
