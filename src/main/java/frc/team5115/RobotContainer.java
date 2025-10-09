package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.Constants.Mode;
import frc.team5115.commands.AutoCommands;
import frc.team5115.subsystems.bling.Bling;
import frc.team5115.subsystems.bling.BlingIO;
import frc.team5115.subsystems.bling.BlingIOReal;
import frc.team5115.subsystems.bling.BlingIOSim;
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
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.intake.IntakeIO;
import frc.team5115.subsystems.intake.IntakeIOSim;
import frc.team5115.subsystems.intake.IntakeIOSparkMax;
import frc.team5115.subsystems.vision.PhotonVision;
import frc.team5115.subsystems.vision.PhotonVisionIO;
import frc.team5115.subsystems.vision.PhotonVisionIOReal;
import frc.team5115.subsystems.vision.PhotonVisionIOSim;
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
    private final Intake intake;
    private final Dealgaefacationinator5000 dealgaefacationinator5000;
    private final Bling bling;

    // Controllers
    private final DriverController driveControl;
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Setings

    private boolean hasFaults = true;
    private double faultPrintTimeout = 0;

    // Works with faults

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        AutoConstants.precomputeAlignmentPoses(); // Computes robot starting pose with vision

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // final GenericEntry dispenseSpeedEntry =
                //         Shuffleboard.getTab("SmartDashboard").add("Dispenser Speed", 0).getEntry();
                //         () -> dispenseSpeedEntry.getDouble(0.1)
                final PneumaticHub hub = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
                gyro = new GyroIONavx();
                climber = new Climber(new ClimberIORev(hub));
                elevator = new Elevator(new ElevatorIOSparkMax());
                dispenser = new Dispenser(new DispenserIOSparkMax(), elevator::getDispenserSpeed);
                intake = new Intake(new IntakeIOSparkMax());
                dealgaefacationinator5000 =
                        new Dealgaefacationinator5000(new Dealgaefacationinator5000IOSparkMax(hub));
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                vision = new PhotonVision(new PhotonVisionIOReal(), drivetrain);
                bling = new Bling(new BlingIOReal());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIO() {};
                climber = new Climber(new ClimberIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                dispenser = new Dispenser(new DispenserIOSim(), elevator::getDispenserSpeed);
                intake = new Intake(new IntakeIOSim());
                dealgaefacationinator5000 =
                        new Dealgaefacationinator5000(new Dealgaefacationinator5000IOSim());
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                vision = new PhotonVision(new PhotonVisionIOSim(), drivetrain);
                bling = new Bling(new BlingIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                climber = new Climber(new ClimberIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                dispenser = new Dispenser(new DispenserIO() {}, elevator::getDispenserSpeed);
                intake = new Intake(new IntakeIO() {});
                dealgaefacationinator5000 =
                        new Dealgaefacationinator5000(new Dealgaefacationinator5000IO() {});
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new PhotonVision(new PhotonVisionIO() {}, drivetrain);
                bling = new Bling(new BlingIO() {});
                break;
        }
        if (Constants.oneController) {
            driveControl =
                    new DriverController(
                            0, drivetrain, dispenser, dealgaefacationinator5000, elevator, climber, intake);
        } else {
            driveControl =
                    new DriverController(
                            0, 1, drivetrain, dispenser, dealgaefacationinator5000, elevator, climber, intake);
        }

        // Register auto commands for pathplanner
        registerCommands(
                drivetrain, vision, elevator, dispenser, intake, dealgaefacationinator5000, climber);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption(
                "Drive Spin SysId (Quasistatic Forward)",
                drivetrain.sysIdSpinQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive Spin SysId (Quasistatic Reverse)",
                drivetrain.sysIdSpinQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive Spin SysId (Dynamic Forward)",
                drivetrain.sysIdSpinDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive Spin SysId (Dynamic Reverse)",
                drivetrain.sysIdSpinDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("Drive All SysIds", drivetrain.driveAllSysIds());

        autoChooser.addOption(
                "Elevator Quasistatic Forward", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Elevator Quasistatic Reverse", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Elevator Dynamic Forward", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Elevator Dynamic Reverse", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        driveControl.configureButtonBindings();
        configureBlingBindings();
    }

    private void configureBlingBindings() {
        bling.setDefaultCommand(bling.redKITT().ignoringDisable(true));
        dispenser.coralDetected().or(elevator.coralDetected()).whileTrue(bling.greenKITT());
        drivetrain.aligningToGoal().whileTrue(bling.yellowScrollIn());
        drivetrain.alignedAtGoalTrigger().whileTrue(bling.whiteScrollIn());
        climber.extended().whileTrue(bling.purpleSolid());
        new Trigger(() -> hasFaults).whileTrue(bling.faultFlash().ignoringDisable(true));
    }

    public void robotPeriodic() {
        if (Constants.currentMode == Mode.REAL) {
            if (faultPrintTimeout <= 0) {
                final var faults =
                        RobotFaults.fromSubsystems(
                                drivetrain,
                                vision,
                                climber,
                                elevator,
                                dispenser,
                                intake,
                                dealgaefacationinator5000,
                                driveControl.isConnected());
                hasFaults = faults.hasFaults();
                if (hasFaults) {
                    System.err.println(faults.toString());
                }
                faultPrintTimeout = 50;
            }
            faultPrintTimeout -= 1;
            Logger.recordOutput("HasFaults", hasFaults);
            Logger.recordOutput("ClearForMatch", !hasFaults);
        }
    }

    /**
     * Register commands for pathplanner to use in autos
     *
     * @param drivetrain
     * @param vision
     * @param elevator
     * @param dispenser
     * @param intake
     * @param dealgaefacationinator5000
     * @param climber
     */
    public static void registerCommands(
            Drivetrain drivetrain,
            PhotonVision vision,
            Elevator elevator,
            Dispenser dispenser,
            Intake intake,
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
                "L2", AutoCommands.testingGetReefAlignCommand(drivetrain, elevator, dispenser, Height.L2));

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

        NamedCommands.registerCommand(
                "Intake", AutoCommands.intakeUntilCoral(dispenser, elevator, intake));
        // Commands for raising while moving auto

        NamedCommands.registerCommand("RaiseElevator", AutoCommands.raiseElevator(elevator, Height.L2));

        NamedCommands.registerCommand(
                "DispenseRight", AutoCommands.scoreSequence(drivetrain, elevator, dispenser, Side.RIGHT));

        NamedCommands.registerCommand(
                "DispenseLeft", AutoCommands.scoreSequence(drivetrain, elevator, dispenser, Side.LEFT));

        NamedCommands.registerCommand(
                "L1",
                AutoCommands.getReefAlignCommand(drivetrain, elevator, dispenser, Side.RIGHT, Height.L1));

        NamedCommands.registerCommand(
                "Clean", AutoCommands.dealgify(drivetrain, elevator, dealgaefacationinator5000));

        NamedCommands.registerCommand(
                "CleanHighL3Left",
                AutoCommands.cleanAndScoreLeft(
                        drivetrain, elevator, dealgaefacationinator5000, dispenser, Height.L4, Height.L3));

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

    public void teleopInit() {
        drivetrain.setTeleopCurrentLimit();
        elevator.zero().schedule();
        // drivetrain.offsetGyro(fRotation2d.fromDegrees(-90));
    }

    public void autoInit() {
        drivetrain.setAutoCurrentLimit();
        // Offset gyro to zero
        drivetrain.offsetGyro();
        // Then offset by 180 degrees
        drivetrain.offsetGyro(Rotation2d.k180deg);
    }

    public void disabledPeriodic() {}
}
