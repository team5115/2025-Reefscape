package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.Constants.Mode;
import frc.team5115.commands.AutoCommands;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.climber.Climber;
import frc.team5115.subsystems.climber.ClimberIO;
import frc.team5115.subsystems.climber.ClimberIORev;
import frc.team5115.subsystems.climber.ClimberIOSim;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000IO;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000IOSim;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000IOSparkMax;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.dispenser.DispenserIO;
import frc.team5115.subsystems.dispenser.DispenserIOSim;
import frc.team5115.subsystems.dispenser.DispenserIOSparkMax;
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
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.indexer.IndexerIO;
import frc.team5115.subsystems.indexer.IndexerIOSim;
import frc.team5115.subsystems.indexer.IndexerIOSparkMax;
import frc.team5115.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;
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
    private final Climber climber;
    private final Elevator elevator;
    private final Dispenser dispenser;
    private final Indexer indexer;
    private final Dealgaefacationinator5000 dealgaefacationinator5000;
    // private final Bling bling;

    // Controllers
    private final CommandXboxController joyDrive = new CommandXboxController(0);
    private final CommandXboxController joyManip = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private boolean robotRelative = false;
    private boolean slowMode = false;
    private boolean hasFaults = true;
    private double faultPrintTimeout = 0;
    private final boolean constantlyCheckFaults = true;

    private final GenericEntry clearForMatchEntry;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        AutoConstants.precomputeAlignmentPoses();
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                final PneumaticHub hub = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
                gyro = new GyroIONavx();
                climber = new Climber(new ClimberIORev(hub));
                elevator = new Elevator(new ElevatorIOSparkMax());
                dispenser = new Dispenser(new DispenserIOSparkMax());
                indexer = new Indexer(new IndexerIOSparkMax(), elevator);
                // bling = new Bling(new BlingIOReal());
                dealgaefacationinator5000 =
                        new Dealgaefacationinator5000(new Dealgaefacationinator5000IOSparkMax(hub));
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                vision = new PhotonVision(drivetrain);
                // vision = null;
                clearForMatchEntry =
                        Shuffleboard.getTab("SmartDashboard").add("ClearForMatch", false).getEntry();
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIO() {};
                climber = new Climber(new ClimberIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                dispenser = new Dispenser(new DispenserIOSim());
                // bling = new Bling(new BlingIOSim());
                indexer = new Indexer(new IndexerIOSim(), elevator);
                dealgaefacationinator5000 =
                        new Dealgaefacationinator5000(new Dealgaefacationinator5000IOSim());
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                vision = null;
                clearForMatchEntry = null;
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                climber = new Climber(new ClimberIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                dispenser = new Dispenser(new DispenserIO() {});
                indexer = new Indexer(new IndexerIO() {}, elevator);
                // bling = new Bling(new BlingIO() {});
                dealgaefacationinator5000 =
                        new Dealgaefacationinator5000(new Dealgaefacationinator5000IO() {});
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = null;
                clearForMatchEntry = null;
                break;
        }

        // Register auto commands for pathplanner
        registerCommands(
                drivetrain, vision, elevator, dispenser, indexer, dealgaefacationinator5000, climber);

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
        joyDrive
                .leftTrigger()
                .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                .whileTrue(drivetrain.alignSelectedSpot(Side.LEFT));
        joyDrive
                .rightTrigger()
                .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                .whileTrue(drivetrain.alignSelectedSpot(Side.RIGHT));
        joyDrive
                .y()
                .onTrue(drivetrain.setRadius())
                .whileTrue(
                        drivetrain.reefOrbitDrive(() -> -joyDrive.getLeftX(), () -> -joyDrive.getLeftY()));

        joyManip.leftStick().whileTrue(elevator.velocityControl(() -> -joyManip.getLeftY() / 10.0));
        elevator.setDefaultCommand(elevator.positionControl());
        joyManip.start().onTrue(elevator.setHeight(Height.MINIMUM));
        joyManip.a().onTrue(elevator.setHeight(Height.INTAKE));
        joyManip.b().onTrue(elevator.setHeight(Height.L2));
        joyManip.y().onTrue(elevator.setHeight(Height.L3));
        joyManip.back().onTrue(elevator.zero()).onTrue(elevator.setHeight(Height.MINIMUM));

        joyManip
                .x()
                .onTrue(dealgaefacationinator5000.extend())
                .onFalse(dealgaefacationinator5000.retract());
        joyManip.rightTrigger().onTrue(dispenser.dispense()).onFalse(dispenser.stop());
        joyManip.leftTrigger().onTrue(dispenser.reverse()).onFalse(dispenser.stop());
        joyManip.rightStick().onTrue(climber.stopCommand());
        joyManip.leftBumper().onTrue(climber.retract());
        joyManip.rightBumper().onTrue(climber.extend());
    }

    private Command setRobotRelative(boolean state) {
        return Commands.runOnce(() -> robotRelative = state);
    }

    private Command setSlowMode(boolean state) {
        return Commands.runOnce(() -> slowMode = state);
    }

    public void robotPeriodic() {}

    /**
     * Register commands for pathplanner to use in autos
     *
     * @param drivetrain
     * @param vision
     * @param elevator
     * @param dispenser
     * @param indexer
     * @param dealgaefacationinator5000
     * @param climber
     */
    public static void registerCommands(
            Drivetrain drivetrain,
            PhotonVision vision,
            Elevator elevator,
            Dispenser dispenser,
            Indexer indexer,
            Dealgaefacationinator5000 dealgaefacationinator5000,
            Climber climber) {
        // Register commands for pathplanner
        NamedCommands.registerCommand(
                "L2Left",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.LEFT, Height.L2));

        NamedCommands.registerCommand(
                "L2Right",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.RIGHT, Height.L2));

        NamedCommands.registerCommand(
                "L3Left",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.LEFT, Height.L3));

        NamedCommands.registerCommand(
                "L3Right",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.RIGHT, Height.L3));

        NamedCommands.registerCommand(
                "L4Left",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.LEFT, Height.L4));

        NamedCommands.registerCommand(
                "L4Right",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.RIGHT, Height.L4));

        NamedCommands.registerCommand("Intake", AutoCommands.intakeUntilCoral(dispenser, elevator));

        System.out.println("Registered Commands");
    }

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

    public void disabledPeriodic() {
        if (Constants.currentMode == Mode.REAL) {
            if (hasFaults || constantlyCheckFaults) {
                if (faultPrintTimeout <= 0) {
                    preMatchCheck();
                    faultPrintTimeout = 50;
                }
                faultPrintTimeout -= 1;
            }
            Logger.recordOutput("HasFaults", hasFaults);
            clearForMatchEntry.setBoolean(!hasFaults);
        }
    }

    private void preMatchCheck() {
        final var faults =
                RobotFaults.fromSubsystems(
                        drivetrain,
                        vision,
                        climber,
                        elevator,
                        dispenser,
                        indexer,
                        null,
                        joyDrive.isConnected() && true); // joyManip.isConnected());
        hasFaults = faults.hasFaults();
        if (hasFaults) {
            System.err.println(faults.toString());
        } else {
            System.out.println(faults.toString());
        }
    }
}
