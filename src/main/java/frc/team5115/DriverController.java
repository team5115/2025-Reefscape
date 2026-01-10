package frc.team5115;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.climber.Climber;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.elevator.Elevator;
import frc.team5115.subsystems.elevator.Elevator.Height;
import frc.team5115.subsystems.intake.Intake;

public class DriverController {
    private final CommandXboxController joyDrive;
    private final CommandXboxController joyManip;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public DriverController() {
        joyDrive = new CommandXboxController(0);
        joyManip = Constants.SINGLE_MODE ? null : new CommandXboxController(1);
    }

    public boolean joysticksConnected() {
        return joyDrive.isConnected() && (Constants.SINGLE_MODE ? true : joyManip.isConnected());
    }

    private Command offsetGyro(Drivetrain drivetrain) {
        return Commands.runOnce(() -> drivetrain.offsetGyro(), drivetrain).ignoringDisable(true);
    }

    public void configureRumbleBindings(
            Drivetrain drivetrain, Dispenser dispenser, Elevator elevator) {
        drivetrain.alignedAtGoalTrigger().onTrue(rumble(Constants.RUMBLE_STRENGTH)).onFalse(rumble(0));
        dispenser
                .coralDetected()
                .or(elevator.coralDetected())
                .debounce(0.5, DebounceType.kFalling)
                .onTrue(rumble(Constants.RUMBLE_STRENGTH))
                .onFalse(rumble(0));
    }

    public void configureButtonBindings(
            Drivetrain drivetrain,
            Dispenser dispenser,
            Dealgaefacationinator5000 dealgae,
            Elevator elevator,
            Climber climber,
            Intake intake) {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));
        if (Constants.SINGLE_MODE) {
            configureSingleMode(drivetrain, dispenser, dealgae, elevator, climber, intake);
        } else {
            configureDualMode(drivetrain, dispenser, dealgae, elevator, climber, intake);
        }
    }

    private void configureSingleMode(
            Drivetrain drivetrain,
            Dispenser dispenser,
            Dealgaefacationinator5000 dealgae,
            Elevator elevator,
            Climber climber,
            Intake intake) {
        /* Drive button bindings -
         * x: forces the robot to stop moving
         * left bumper: Sets robot relative to true while held down
         * right bumper: Sets slow mode while held down
         * left and right triggers align to score respectively
         * both triggers aligns to the middle
         * start resets field orientation
         */

        // joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        // joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        // joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));

        joyDrive.start().onTrue(offsetGyro(drivetrain));

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                .whileTrue(drivetrain.alignSelectedSpot());
        joyDrive
                .rightTrigger()
                .and(joyDrive.leftTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                .whileTrue(drivetrain.alignSelectedSpot());

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.CENTER))
                .whileTrue(drivetrain.alignSelectedSpot());

        /*
        * Manipulator button bindings:
        * hold left stick and move it for elevator manual control
        * hold start for L1
        * hold b for L2
        * hold x for L3
        * press back to rezero elevator
        * hold a to vomit
        * hold right trigger to dispense
        * hold left trigger to reverse dispense
        * press right bumper to extend climb piston
        * press left bumper to retract climb piston
        * point up on dpad to toggle climber block
        //  * point down on dpad and press B (L2) or X (L3) to clean algae, release to stow
        */

        // divide by 100 to achieve 3 cm/s max speed
        elevator.setDefaultCommand(elevator.positionControl());

        // driver holds down a, manip controls elevator velocity
        // joyDrive.a().whileTrue(elevator.velocityControl(() -> -joyManip.getLeftY() * 0.5));

        intake.setDefaultCommand(intake.intakeIf(elevator::atIntake));

        // joyManip
        //         .start()
        //         .onTrue(elevator.setHeight(Height.L1))
        //         .onFalse(elevator.setHeight(Height.INTAKE));
        joyDrive.b().onTrue(elevator.setHeight(Height.L2)).onFalse(elevator.setHeight(Height.INTAKE));
        joyDrive.x().onTrue(elevator.setHeight(Height.L3)).onFalse(elevator.setHeight(Height.INTAKE));

        joyDrive
                .y()
                .onTrue(elevator.setHeight(Height.CLEAN3))
                .onFalse(elevator.setHeight(Height.INTAKE));

        joyDrive.back().onTrue(elevator.zero()).onFalse(elevator.setHeight(Height.MINIMUM));

        joyDrive.rightBumper().onTrue(dispenser.dispense()).onFalse(dispenser.stop());
        joyDrive
                .leftBumper()
                .whileTrue(intake.vomit().repeatedly().alongWith(dispenser.reverse().repeatedly()))
                .onFalse(intake.stop().alongWith(dispenser.stop()));
        // joyManip.leftTrigger().onTrue(dispenser.reverse()).onFalse(dispenser.stop());
        // joyManip.pov(180).onTrue(dispenser.altDispense()).onFalse(dispenser.stop());

        // joyManip.rightBumper().onTrue(climber.extend());
        // joyManip.leftBumper().onTrue(climber.retract());
        // joyManip.pov(0).onTrue(climber.toggleShield());

        joyDrive
                .pov(180)
                .or(joyDrive.pov(135))
                .or(joyDrive.pov(225))
                .onTrue(dealgae.prepClean())
                .onFalse(dealgae.completeClean());

        joyDrive.povRight().onTrue(climber.extend());
        joyDrive.povLeft().onTrue(climber.retract());
        joyDrive.povUp().onTrue(climber.toggleShield());
    }

    private void configureDualMode(
            Drivetrain drivetrain,
            Dispenser dispenser,
            Dealgaefacationinator5000 dealgae,
            Elevator elevator,
            Climber climber,
            Intake intake) {
        /* Drive button bindings -
         * x: forces the robot to stop moving
         * left bumper: Sets robot relative to true while held down
         * right bumper: Sets slow mode while held down
         * left and right triggers align to score respectively
         * both triggers aligns to the middle
         * start resets field orientation
         */

        joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));
        joyDrive.start().onTrue(offsetGyro(drivetrain));

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                .whileTrue(drivetrain.alignSelectedSpot());
        joyDrive
                .rightTrigger()
                .and(joyDrive.leftTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                .whileTrue(drivetrain.alignSelectedSpot());

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.CENTER))
                .whileTrue(drivetrain.alignSelectedSpot());

        /*
        * Manipulator button bindings:
        * hold left stick and move it for elevator manual control
        * hold start for L1
        * hold b for L2
        * hold x for L3
        * press back to rezero elevator
        * hold a to vomit
        * hold right trigger to dispense
        * hold left trigger to reverse dispense
        * press right bumper to extend climb piston
        * press left bumper to retract climb piston
        * point up on dpad to toggle climber block
        //  * point down on dpad and press B (L2) or X (L3) to clean algae, release to stow
        */

        // divide by 100 to achieve 3 cm/s max speed
        elevator.setDefaultCommand(elevator.positionControl());

        // driver holds down a, manip controls elevator velocity
        joyDrive.a().whileTrue(elevator.velocityControl(() -> -joyManip.getLeftY() * 0.5));

        intake.setDefaultCommand(intake.intakeIf(elevator::atIntake));
        joyManip
                .a()
                .whileTrue(intake.vomit().repeatedly().alongWith(dispenser.reverse().repeatedly()))
                .onFalse(intake.stop().alongWith(dispenser.stop()));

        joyManip
                .start()
                .onTrue(elevator.setHeight(Height.L1))
                .onFalse(elevator.setHeight(Height.INTAKE));
        joyManip.b().onTrue(elevator.setHeight(Height.L2)).onFalse(elevator.setHeight(Height.INTAKE));
        joyManip.x().onTrue(elevator.setHeight(Height.L3)).onFalse(elevator.setHeight(Height.INTAKE));

        joyManip
                .y()
                .onTrue(elevator.setHeight(Height.CLEAN3))
                .onFalse(elevator.setHeight(Height.INTAKE));

        joyManip.back().onTrue(elevator.zero()).onFalse(elevator.setHeight(Height.MINIMUM));

        joyManip.rightTrigger().onTrue(dispenser.dispense()).onFalse(dispenser.stop());
        joyManip.leftTrigger().onTrue(dispenser.reverse()).onFalse(dispenser.stop());
        // joyManip.pov(180).onTrue(dispenser.altDispense()).onFalse(dispenser.stop());

        joyManip.rightBumper().onTrue(climber.extend());
        joyManip.leftBumper().onTrue(climber.retract());
        joyManip.pov(0).onTrue(climber.toggleShield());

        joyManip
                .pov(180)
                .or(joyManip.pov(135))
                .or(joyManip.pov(225))
                .onTrue(dealgae.prepClean())
                .onFalse(dealgae.completeClean());
        // .onFalse(dealgaefacationinator5000.clean());
    }

    private Command setRobotRelative(boolean state) {
        return Commands.runOnce(() -> robotRelative = state);
    }

    private Command setSlowMode(boolean state) {
        return Commands.runOnce(() -> slowMode = state);
    }

    public boolean getRobotRelative() {
        return robotRelative;
    }

    public boolean getSlowMode() {
        return slowMode;
    }

    private Command rumble(double value) {
        return Commands.runOnce(
                () -> {
                    joyDrive.setRumble(GenericHID.RumbleType.kBothRumble, value);
                    if (joyManip != null) {
                        joyManip.setRumble(GenericHID.RumbleType.kBothRumble, value);
                    }
                });
    }
}
