package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.climber.Climber;
import frc.team5115.subsystems.climber.ClimberIO;
import frc.team5115.subsystems.climber.ClimberIOSim;
import frc.team5115.subsystems.climber.ClimberIOSparkMax;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.drive.GyroIO;
import frc.team5115.subsystems.drive.GyroIONavx;
import frc.team5115.subsystems.drive.ModuleIO;
import frc.team5115.subsystems.drive.ModuleIOSim;
import frc.team5115.subsystems.drive.ModuleIOSparkMax;
import frc.team5115.subsystems.elevator.Elevator;
import frc.team5115.subsystems.elevator.Elevator.Height;
import frc.team5115.subsystems.elevator.ElevatorIO;
import frc.team5115.subsystems.elevator.ElevatorIOSim;
import frc.team5115.subsystems.elevator.ElevatorIOSparkMax;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.intake.IntakeIO;
import frc.team5115.subsystems.intake.IntakeIOSim;
import frc.team5115.subsystems.intake.IntakeIOSparkMax;
import frc.team5115.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final GyroIO gyro;
    private final Drivetrain drivetrain;
    private final PhotonVision vision;
    private final Intake intake;
    private final Climber climber;
    private final Elevator elevator;

    // Controllers
    private final CommandXboxController joyDrive = new CommandXboxController(0);
    private final CommandXboxController joyManip = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Shuffleboard
    private boolean robotRelative = false;
    private boolean slowMode = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                gyro = new GyroIONavx();
                intake = new Intake(new IntakeIOSparkMax());
                climber = new Climber(new ClimberIOSparkMax());
                elevator = new Elevator(new ElevatorIOSparkMax());
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                vision = new PhotonVision(drivetrain);
                // vision = null;
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIO() {};
                intake = new Intake(new IntakeIOSim());
                climber = new Climber(new ClimberIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                vision = null;
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                intake = new Intake(new IntakeIO() {});
                climber = new Climber(new ClimberIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = null;
                break;
        }

        // Register auto commands for pathplanner
        // PhotonVision is passed in here to prevent warnings, i.e. "unused variable: vision"
        registerCommands(drivetrain, vision, intake);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)",
        //         drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)",
        //         drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Forward)",
        //         drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Reverse)",
        //         drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Shooter SysId (Quasistatic Forward)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Shooter SysId (Quasistatic Reverse)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Shooter SysId (Dynamic Forward)",
        //         shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Shooter SysId (Dynamic Reverse)",
        //         shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));

        joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));

        joyDrive.start().onTrue(resetFieldOrientation());

        joyDrive.a().onTrue(elevator.setHeight(Height.BOTTOM));
        joyDrive.b().onTrue(elevator.setHeight(Height.MIDDLE));
        joyDrive.y().onTrue(elevator.setHeight(Height.TOP));

        // manip control
        climber.setDefaultCommand(climber.climbBy(() -> joyManip.getLeftY()));

        // joyManip
        //         .rightBumper()
        //         .onTrue(
        //                 Commands.sequence(
        //                         DriveCommands.automaticallyPrepareShoot(drivetrain, arm, intake,
        // feeder, shooter),
        //                         DriveCommands.feed(intake, feeder),
        //                         shooter.stop()))
        //         .onFalse(DriveCommands.stow(arm, intake, feeder, shooter));
    }

    private Command setRobotRelative(boolean state) {
        return Commands.runOnce(() -> robotRelative = state);
    }

    private Command setSlowMode(boolean state) {
        return Commands.runOnce(() -> slowMode = state);
    }

    public void robotPeriodic() {}

    /**
     * Registers commands for pathplanner to use in autos
     *
     * @param shooter the shooter subsystem
     * @param arm the arm subsystem
     * @param drivetrain the drivetrain subsytem (not currently used)
     * @param photonVision the photonvision subsystem (not currently used)
     */
    public static void registerCommands(Drivetrain drivetrain, PhotonVision vision, Intake intake) {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private Command resetFieldOrientation() {
        return Commands.runOnce(
                        () -> {
                            drivetrain.setPose(
                                    new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d()));
                            drivetrain.offsetGyro();
                        },
                        drivetrain)
                .ignoringDisable(true);
    }

    private Command resetRobotPose() {
        return Commands.runOnce(
                        () -> {
                            drivetrain.setPose(new Pose2d(new Translation2d(8.5, 6.0), new Rotation2d()));
                        },
                        drivetrain)
                .ignoringDisable(true);
    }
}
