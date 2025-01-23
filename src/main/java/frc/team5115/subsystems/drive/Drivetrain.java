package frc.team5115.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.util.LocalADStarAK;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;

    // TODO tune drive pids
    private final double linear_kp = 1.9;
    private final double linear_ki = 0.0;
    private final double linear_kd = 0.0;
    private final double angular_kp = 0.7;
    private final double angular_ki = 0.0;
    private final double angular_kd = 0.0;

    private final ProfiledPIDController anglePid =
            new ProfiledPIDController(
                    angular_kp * SwerveConstants.MAX_ANGULAR_SPEED,
                    angular_ki,
                    angular_kd,
                    new TrapezoidProfile.Constraints(
                            SwerveConstants.MAX_ANGULAR_SPEED, SwerveConstants.MAX_ANGULAR_SPEED * 2));
    private final ProfiledPIDController xPid =
            new ProfiledPIDController(
                    linear_kp,
                    linear_ki,
                    linear_kd,
                    new TrapezoidProfile.Constraints(
                            SwerveConstants.MAX_LINEAR_SPEED, SwerveConstants.MAX_LINEAR_SPEED * 2));
    private final ProfiledPIDController yPid =
            new ProfiledPIDController(
                    linear_kp,
                    linear_ki,
                    linear_kd,
                    new TrapezoidProfile.Constraints(
                            SwerveConstants.MAX_LINEAR_SPEED, SwerveConstants.MAX_LINEAR_SPEED * 2));

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics,
                    rawGyroRotation,
                    lastModulePositions,
                    new Pose2d(),
                    VecBuilder.fill(0.1, 0.1, 0.1),
                    VecBuilder.fill(0.9, 0.9, 0.9));

    public Drivetrain(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        anglePid.enableContinuousInput(-Math.PI, Math.PI);

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                (var speeds, var feedforwards) -> runVelocity(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(linear_kp, linear_ki, linear_kd),
                        new PIDConstants(angular_kp, angular_ki, angular_kd)),
                new RobotConfig(
                        SwerveConstants.ROBOT_MASS,
                        SwerveConstants.ROBOT_MOI,
                        new ModuleConfig(
                                SwerveConstants.WHEEL_RADIUS_METERS,
                                SwerveConstants.MAX_LINEAR_SPEED,
                                SwerveConstants.WHEEL_COF,
                                DCMotor.getNEO(1),
                                SwerveConstants.DrivingMotorReduction,
                                SwerveConstants.DrivingMotorCurrentLimit, // less than the real current limit
                                1),
                        SwerveConstants.MODULE_TRANSLATIONS),
                () -> false,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    for (int i = 0; i < 4; i++) {
                                        modules[i].runCharacterization(voltage.baseUnitMagnitude());
                                    }
                                },
                                null,
                                this));
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                            modulePositions[moduleIndex].distanceMeters
                                    - lastModulePositions[moduleIndex].distanceMeters,
                            modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs.connected) {
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.yawPosition;
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply odometry update
        poseEstimator.update(rawGyroRotation, modulePositions);
    }

    public Command driveToPosition(Pose2d goalPose, Supplier<Pose2d> poseSupplier) {
        return setAutoAimPids(goalPose)
                .andThen(driveByAutoAimPids(poseSupplier))
                .until(() -> anglePid.atSetpoint() && xPid.atSetpoint() && yPid.atSetpoint());
    }

    private Command setAutoAimPids(Pose2d goalPose) {
        return Commands.runOnce(
                () -> {
                    // ? Could do some more processing here for alliance specific stuff
                    xPid.setGoal(goalPose.getX());
                    yPid.setGoal(goalPose.getY());
                    anglePid.setGoal(goalPose.getRotation().getRadians());
                },
                this);
    }

    private Command driveByAutoAimPids(Supplier<Pose2d> poseSupplier) {
        return Commands.runEnd(
                () -> {
                    final var pose = poseSupplier.get();
                    final var omega = anglePid.calculate(pose.getRotation().getRadians());
                    final var xVelocity = xPid.calculate(pose.getX());
                    final var yVelocity = yPid.calculate(pose.getY());

                    Logger.recordOutput("AutoAim/xVelocity", xVelocity);
                    Logger.recordOutput("AutoAim/yVelocity", yVelocity);
                    Logger.recordOutput("AutoAim/omega", omega);
                    Logger.recordOutput(
                            "AutoAim/Goal",
                            new Pose2d(
                                    new Translation2d(xPid.getGoal().position, yPid.getGoal().position),
                                    new Rotation2d(anglePid.getGoal().position)));

                    runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getRotation()));
                },
                this::stop,
                this);
    }

    // public Command dirve

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("ChassisSpeedsDiscrete", speeds);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = SwerveConstants.MODULE_TRANSLATIONS[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Rotation2d getGyroRotation() {
        if (gyroInputs.connected) {
            return gyroInputs.yawPosition.minus(gyroOffset);
        } else {
            return getRotation();
        }
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    private Rotation2d gyroOffset = new Rotation2d();

    public void offsetGyro() {
        gyroOffset = rawGyroRotation;
    }
}
