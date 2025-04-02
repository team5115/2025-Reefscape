package frc.team5115.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import frc.team5115.Constants.ElevatorConstants;
import frc.team5115.subsystems.dispenser.Dispenser;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
    private static final double minHeightInches = 22;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController fastPid; // control meters, output m/s
    private final ProfiledPIDController slowPid; // control meters, output m/s
    private ProfiledPIDController selectedPid;
    private final ElevatorFeedforward feedforward;
    private final SysIdRoutine sysId;
    private Height height = Height.MINIMUM; // Start at the minimum height

    @AutoLogOutput private double velocitySetpoint;

    @AutoLogOutput public double offset;

    @AutoLogOutput
    private final LoggedMechanism2d elevatorMechanism2d =
            new LoggedMechanism2d(10, Height.L4.position * 10);

    private final LoggedMechanismRoot2d elevatorMechanismRoot2d =
            elevatorMechanism2d.getRoot(getName() + " Root", 0, 0);
    private final LoggedMechanismLigament2d elevatorMechanismLigament2d =
            elevatorMechanismRoot2d.append(new LoggedMechanismLigament2d(getName(), 0, 90));

    public enum Height {
        MINIMUM(minHeightInches),
        INTAKE(minHeightInches),
        L1(minHeightInches + 7.0),
        L2(minHeightInches + 15.85),
        L3(55),
        CLEAN3(63),
        L4(63);

        public final double position; // meters

        Height(double inches) {
            position = (inches - minHeightInches) * 0.0254;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
        elevatorMechanismLigament2d.append(new LoggedMechanismLigament2d(getName() + "2", 10, -90));
        SmartDashboard.putData("Elevator Mechanism", elevatorMechanism2d);
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                fastPid =
                        new ProfiledPIDController(
                                ElevatorConstants.KP,
                                ElevatorConstants.KI,
                                ElevatorConstants.KD,
                                new TrapezoidProfile.Constraints(
                                        ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACCEL));
                slowPid =
                        new ProfiledPIDController(
                                ElevatorConstants.KP * ElevatorConstants.SLOW_CONSTANT,
                                ElevatorConstants.KI * ElevatorConstants.SLOW_CONSTANT,
                                ElevatorConstants.KD * ElevatorConstants.SLOW_CONSTANT,
                                new TrapezoidProfile.Constraints(
                                        ElevatorConstants.MAX_VEL * ElevatorConstants.SLOW_CONSTANT,
                                        ElevatorConstants.MAX_ACCEL * ElevatorConstants.SLOW_CONSTANT));
                feedforward =
                        new ElevatorFeedforward(
                                ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
                break;
            case SIM:
                fastPid =
                        new ProfiledPIDController(
                                1.0,
                                0.0,
                                0.0,
                                new TrapezoidProfile.Constraints(
                                        ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACCEL));
                slowPid =
                        new ProfiledPIDController(
                                1.0 * ElevatorConstants.SLOW_CONSTANT,
                                0.0,
                                0.0,
                                new TrapezoidProfile.Constraints(
                                        ElevatorConstants.MAX_VEL * ElevatorConstants.SLOW_CONSTANT,
                                        ElevatorConstants.MAX_ACCEL * ElevatorConstants.SLOW_CONSTANT));
                feedforward =
                        new ElevatorFeedforward(
                                ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
                break;
            default:
                fastPid =
                        new ProfiledPIDController(
                                0.0,
                                0.0,
                                0.0,
                                new TrapezoidProfile.Constraints(
                                        ElevatorConstants.MAX_VEL, ElevatorConstants.MAX_ACCEL));
                slowPid =
                        new ProfiledPIDController(
                                0.0,
                                0.0,
                                0.0,
                                new TrapezoidProfile.Constraints(
                                        ElevatorConstants.MAX_VEL * ElevatorConstants.SLOW_CONSTANT,
                                        ElevatorConstants.MAX_ACCEL * ElevatorConstants.SLOW_CONSTANT));
                feedforward = new ElevatorFeedforward(0, 0, 0);
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
        fastPid.setTolerance(0.05);
        slowPid.setTolerance(0.05);
        fastPid.setGoal(height.position);
        slowPid.setGoal(height.position);
        selectedPid = fastPid;
    }

    @AutoLogOutput
    public double getActualHeight() {
        return inputs.positionMeters + offset;
    }

    @AutoLogOutput
    public double getInchesFromGround() {
        return getActualHeight() * 100d / 2.54 + minHeightInches;
    }

    @AutoLogOutput
    public boolean isShorting() {
        return (inputs.magnet1detected && inputs.magnet2detected && inputs.magnet3detected);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        Logger.recordOutput("Elevator/Goal Height", height.position);
        Logger.recordOutput("Elevator/Actual Velocity", inputs.velocityMetersPerSecond);

        if (!isShorting()) {
            if (inputs.magnet1detected) {
                offset = ElevatorConstants.FIRST_MAGNET_HEIGHT - inputs.positionMeters;
            }
            // ! Shut off the upper magnets because they cause "drift"
            // if (inputs.magnet2detected) {
            //     offset = ElevatorConstants.SECOND_MAGNET_HEIGHT - inputs.positionMeters;
            // }
            // if (inputs.magnet3detected) {
            //     offset = ElevatorConstants.THIRD_MAGNET_HEIGHT - inputs.positionMeters;
            // }
        }

        // if (inputs.magnet4detected) {
        //     offset = fourthHeight - inputs.positionMeters;
        // }
        // if (inputs.backCoralDetected) {
        //     // Force the elvator to stay at the intake position when there is a coral in the intake
        //     height = Height.INTAKE;
        // }
        // io.setElevatorVelocity(velocitySetpoint, 0.0);

        final double volts = feedforward.calculate(velocitySetpoint);
        if (Math.abs(volts) <= ElevatorConstants.KG) {
            io.setElevatorVoltage(0);
        } else {
            io.setElevatorVoltage(volts);
        }
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
                                setVelocity(-0.5),
                                Commands.either(waitUntilStall(), waitUntilMagnet1(), this::isShorting),
                                zeroHere())
                        .unless(() -> inputs.magnet1detected && !isShorting());
        retval.addRequirements(this);
        return retval;
    }

    private Command waitUntilStall() {
        return Commands.waitUntil(
                () -> Math.abs(inputs.velocityMetersPerSecond) < 0.01 && inputs.currentAmps > 30.0);
    }

    private Command waitUntilMagnet1() {
        return Commands.waitUntil(() -> inputs.magnet1detected == true);
    }

    private Command zeroHere() {
        return Commands.runOnce(
                () -> {
                    velocitySetpoint = 0;
                    offset = -inputs.positionMeters;
                },
                this);
    }

    private Command setVelocity(double v) {
        return Commands.runOnce(() -> velocitySetpoint = v, this);
    }

    public Command waitForSetpoint(double timeout) {
        return Commands.waitUntil(() -> atGoal()).withTimeout(timeout);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.backCoralDetected == state).withTimeout(timeout);
    }

    public boolean atIntake() {
        return atGoal() && selectedPid.getGoal().position == Height.INTAKE.position;
    }

    public Command setHeight(Height height) {
        return Commands.runOnce(
                () -> {
                    this.height = height;
                    selectedPid.setGoal(height.position);
                });
    }

    public Command setHeightAndWait(Height height, double timeoutSeconds) {
        return setHeight(height).andThen(waitForSetpoint(timeoutSeconds));
    }

    @AutoLogOutput(key = "State")
    private String getStateString() {
        if (atGoal()) {
            return height.toString();
        } else {
            return "MOVING_TO_" + height.toString();
        }
    }

    @AutoLogOutput
    public boolean atGoal() {
        return Math.abs(velocitySetpoint - inputs.velocityMetersPerSecond) <= 0.1
                && selectedPid.atSetpoint();
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
                    if (velocitySetpoint < 0 && inputs.magnet1detected && !isShorting()) {
                        // If we are moving down and we are at the bottom we stop and don't run pids
                        velocitySetpoint = 0;
                    } else {
                        // Otherwise, we select a pid controller and calculate the velocity setpoint by
                        // if (velocitySetpoint < 0
                        //         && getActualHeight() <= ElevatorConstants.SLOW_PID_HEIGHT_METERS) {
                        //     // use slow pid when we are moving down and below a certain height
                        //     // selectedPid = slowPid;
                        // } else {
                        //     selectedPid = fastPid;
                        // }
                        selectedPid = fastPid;
                        velocitySetpoint = selectedPid.calculate(getActualHeight(), height.position);
                        if (height.position == Height.INTAKE.position
                                && !inputs.magnet1detected
                                && Math.abs(velocitySetpoint) < 0.05
                                && !isShorting()) {
                            // if at intake but magnet 1 not detected yet, then we keep going down
                            velocitySetpoint = -0.5;
                        }
                    }
                },
                this);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }

    public double getDispenserSpeed() {
        if (getActualHeight() <= (Height.L2.position + Height.L1.position) / 2) {
            return Dispenser.l1Speed;
        } else if (getActualHeight() >= (Height.L3.position + Height.L4.position) / 2) {
            return Dispenser.l4Speed;
        } else {
            return Dispenser.normalSpeed;
        }
    }
}
