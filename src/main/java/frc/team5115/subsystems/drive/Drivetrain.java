package frc.team5115.subsystems.drive;

import static edu.wpi.first.units.measure.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;

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

    private final double lateralConstantP = 1.9;
    private final double angleConstantP = 0.7;
    private final PIDController anglePid =
            new PIDController(angleConstantP * SwerveConstants.MAX_ANGULAR_SPEED, 0, 0);
    private final PIDController xPid = new PIDController(lateralConstantP, 0, 0);
    private final PIDController yPid = new PIDController(lateralConstantP, 0, 0);

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

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(
                        SwerveConstants.MAX_LINEAR_SPEED,
                        SwerveConstants.DRIVE_BASE_RADIUS,
                        new ReplanningConfig()),
                this::isRedAlliance,
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
                                        modules[i].runCharacterization(voltage.in(Volts));
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

    public Command faceSpeaker() {
        return setAutoAimPids()
                .andThen(driveByAutoAimPids())
                .until(() -> anglePid.atSetpoint() && xPid.atSetpoint() && yPid.atSetpoint());
    }

    private Command driveByAutoAimPids() {
        return Commands.runEnd(
                () -> {
                    final var omega = anglePid.calculate(getPose().getRotation().getRadians());
                    final var xVelocity = xPid.calculate(getPose().getX());
                    final var yVelocity = yPid.calculate(getPose().getY());

                    Logger.recordOutput("AutoAim/xVelocity", xVelocity);
                    Logger.recordOutput("AutoAim/yVelocity", yVelocity);
                    Logger.recordOutput("AutoAim/omega", omega);
                    Logger.recordOutput(
                            "AutoAim/Setpoint", new Translation2d(xPid.getSetpoint(), yPid.getSetpoint()));

                    runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getRotation()));
                },
                this::stop,
                this);
    }

    private Command setAutoAimPids() {
        return Commands.runOnce(
                () -> {
                    double blueSpeakerXMeters = 0.96;
                    if (isRedAlliance()) {
                        blueSpeakerXMeters = Constants.FIELD_WIDTH_METERS - blueSpeakerXMeters;
                    }
                    final var speakerX = Meters.of(blueSpeakerXMeters);
                    final var speakerY = Meters.of(5.536);
                    final var distanceForShot = Feet.of(10);
                    final Translation2d speaker = new Translation2d(speakerX, speakerY);
                    final Translation2d robot = getPose().getTranslation();
                    final Translation2d robotToSpeaker = speaker.minus(robot);
                    final double distanceToSpeaker = robot.getDistance(speaker);
                    final double moveDelta = distanceToSpeaker - distanceForShot.in(Meter);
                    final Rotation2d theta =
                            Rotation2d.fromRadians(
                                    MathUtil.angleModulus(Math.atan2(robotToSpeaker.getY(), robotToSpeaker.getX())));
                    final double deltaX = moveDelta * theta.getCos();
                    final double deltaY = moveDelta * theta.getSin();
                    final double setpointX = robot.getX() + deltaX;
                    final double setpointY = robot.getY() + deltaY;
                    anglePid.setSetpoint(theta.plus(Rotation2d.fromDegrees(180)).getRadians());
                    xPid.setSetpoint(setpointX);
                    yPid.setSetpoint(setpointY);
                },
                this);
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }

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
        Logger.recordOutput("ChassisSpeedsDiscrete", discreteSpeeds);
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
