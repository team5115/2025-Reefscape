package frc.team5115;

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
import frc.team5115.subsystems.vision.PhotonVision;

public class DriverController {
    private final CommandXboxController joyDrive;
    private final CommandXboxController joyManip;

    private final Drivetrain drivetrain;
    private final Climber climber;
    private final Elevator elevator;
    private final Dispenser dispenser;
    private final Intake intake;
    private final Dealgaefacationinator5000 dealgaefacationinator5000;
    private boolean robotRelative = false;
    private boolean slowMode = false;

    public DriverController(int port,Drivetrain drivetrain,Dispenser dispenser,Dealgaefacationinator5000 dealgae,Elevator elevator,Climber climber,Intake intake){
        joyDrive = new CommandXboxController(port);
        joyManip = null;

        this.drivetrain = drivetrain;
        this.climber = climber;
        this.dealgaefacationinator5000 = dealgae;
        this.dispenser = dispenser;
        this.elevator = elevator;
        this.intake = intake;
    }
    public DriverController(int drivePort, int manipPort, Drivetrain drivetrain,Dispenser dispenser,Dealgaefacationinator5000 dealgae,Elevator elevator,Climber climber,Intake intake){
        joyDrive = new CommandXboxController(drivePort);
        joyManip = new CommandXboxController(manipPort);

        this.drivetrain = drivetrain;
        this.climber = climber;
        this.dealgaefacationinator5000 = dealgae;
        this.dispenser = dispenser;
        this.elevator = elevator;
        this.intake = intake;
    }
    
    private Command offsetGyro() {
        return Commands.runOnce(() -> drivetrain.offsetGyro(), drivetrain).ignoringDisable(true);
    }
    public boolean isConnected(){
        return joyDrive.isConnected() && (joyManip == null || joyManip.isConnected());
    }

    public void configureButtonBindings(){
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));
        if(joyManip == null){
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
            
            joyDrive.start().onTrue(offsetGyro());

            joyDrive
                    .leftTrigger()
                    .and(joyDrive.rightTrigger().negate())
                    .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                    .whileTrue(drivetrain.alignSelectedSpot(Side.LEFT));
            joyDrive
                    .rightTrigger()
                    .and(joyDrive.leftTrigger().negate())
                    .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                    .whileTrue(drivetrain.alignSelectedSpot(Side.RIGHT));

            joyDrive
                    .leftTrigger()
                    .and(joyDrive.rightTrigger())
                    .onTrue(drivetrain.selectNearestScoringSpot(Side.CENTER))
                    .whileTrue(drivetrain.alignSelectedSpot(Side.CENTER));

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
                    .onTrue(dealgaefacationinator5000.prepClean())
                    .onFalse(dealgaefacationinator5000.completeClean());
        } else {
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
             //joyDrive.start().onTrue(offsetGyro());
 
             joyDrive
                     .leftTrigger()
                     .and(joyDrive.rightTrigger().negate())
                     .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                     .whileTrue(drivetrain.alignSelectedSpot(Side.LEFT));
             joyDrive
                     .rightTrigger()
                     .and(joyDrive.leftTrigger().negate())
                     .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                     .whileTrue(drivetrain.alignSelectedSpot(Side.RIGHT));
 
             joyDrive
                     .leftTrigger()
                     .and(joyDrive.rightTrigger())
                     .onTrue(drivetrain.selectNearestScoringSpot(Side.CENTER))
                     .whileTrue(drivetrain.alignSelectedSpot(Side.CENTER));
 
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
                     .onTrue(dealgaefacationinator5000.prepClean())
                     .onFalse(dealgaefacationinator5000.completeClean());
             // .onFalse(dealgaefacationinator5000.clean());
        }
        
    }

    private Command setRobotRelative(boolean state) {
        return Commands.runOnce(() -> robotRelative = state);
    }

    private Command setSlowMode(boolean state) {
        return Commands.runOnce(() -> slowMode = state);
    }
    public boolean getRobotRelative(){
        return robotRelative;
    }
    public boolean getSlowMode(){
        return slowMode;
    }
}
