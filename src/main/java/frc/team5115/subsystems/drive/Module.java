package frc.team5115.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team5115.Constants;
import frc.team5115.Constants.SwerveConstants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0.014011, 0.09899, 0.0041833);
                driveFeedback = new PIDController(0.05, 0.0, 0.0);
                turnFeedback = new PIDController(2.4, 0.0, 0.0);
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Run closed loop turn control
        if (angleSetpoint != null) {
            double turnVoltage =
                    turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians());
            io.setTurnVoltage(turnVoltage);

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
                // Scale velocity based on turn error
                //
                // When the error is 90 degrees, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getError());

                // Run drive controller
                double velocityRadPerSec = adjustSpeedSetpoint / SwerveConstants.WHEEL_RADIUS_METERS;
                double driveVoltage =
                        driveFeedforward.calculate(velocityRadPerSec)
                                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec);
                io.setDriveVoltage(driveVoltage);

                // Logger.recordOutput("ModuleVoltage/DriveVoltage/Index" + Integer.toString(index),
                // driveVoltage);
                // Logger.recordOutput("ModuleVoltage/TurnVoltage/Index" + Integer.toString(index),
                // turnVoltage);
            }
        }
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        state.optimize(getAngle());
        // Update setpoints, controllers run in "periodic" when setpoints are not null
        angleSetpoint = state.angle;
        speedSetpoint = state.speedMetersPerSecond;

        return state;
    }

    /** Runs the module with the specified voltage while controlling to zero degrees. */
    public void runCharacterization(double volts) {
        // Closed loop turn control
        angleSetpoint = new Rotation2d();

        // Open loop drive control
        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    /** Runs the module with the specified voltage while controlling to the given angle */
    public void runCharacterization(double volts, Rotation2d angle) {
        // Closed loop turn loop control
        angleSetpoint = angle;

        // Open loop drive control
        io.setDriveVoltage(volts);
        speedSetpoint = null;
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * SwerveConstants.WHEEL_RADIUS_METERS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS_METERS;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    public void setDriveCurrentLimit(int amps) {
        io.setDriveCurrentLimit(amps);
    }

    /**
     * Get the drive and turn sparks into a list
     *
     * @param sparks the arraylist to append the sparks into
     */
    public void getAllSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
