package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.subsystems.dealgaefacationinator5000.Dealgaefacationinator5000;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.elevator.Elevator;
import frc.team5115.subsystems.elevator.Elevator.Height;
import frc.team5115.subsystems.intake.Intake;

public class AutoCommands {
    private AutoCommands() {}

    // Move elevator to intake position, and then wait until the coral has been fully indexed
    public static Command intakeUntilCoral(Dispenser dispenser, Elevator elevator, Intake intake) {
        // return Commands.print("Intaking").andThen(Commands.waitSeconds(1));
        return Commands.sequence(
                Commands.print("Intaking"),
                elevator.setHeight(Elevator.Height.INTAKE),
                intake.intake(),
                elevator.waitForDetectionState(true, 5.0),
                dispenser.waitForDetectionState(true, 1.0));
    }

    // Move elevator to state parameter and then dispense until coral fully exits
    // Then, move back to intake height preemptively
    public static Command dispense(Dispenser dispenser, Elevator elevator, Elevator.Height state) {
        // return Commands.print("Dispensing").andThen(Commands.waitSeconds(1));
        return Commands.sequence(
                Commands.print("Dispensing"),
                elevator.setHeightAndWait(state, 3.0),
                dispenser.dispense().withTimeout(0.75),
                // dispenser.waitForDetectionState(false, 0.75),
                // Commands.waitSeconds(0.75),
                Commands.print("Stopping Dispenser after Dispense"),
                dispenser.stop(),
                elevator.setHeight(Elevator.Height.INTAKE));
    }
    // without raising while aligning
    // public static Command getReefAlignCommand(
    //         Drivetrain drivetrain, Elevator elevator, Dispenser dispenser, Side side, Height
    // height) {
    //     return Commands.sequence(
    //             elevator.setHeight(height),
    //             drivetrain.autoAlignToScoringSpot(side),
    //             dispense(dispenser, elevator, height));
    // }

    // command for raising while aligning
    public static Command getReefAlignCommand(
            Drivetrain drivetrain, Elevator elevator, Dispenser dispenser, Side side, Height height) {
        return Commands.sequence(
                Commands.parallel(
                        elevator.setHeight(height), drivetrain.autoAlignToScoringSpot(side).withTimeout(2.0)),
                dispense(dispenser, elevator, height));
    }

    public static Command raiseElevator(Elevator elevator, Height height) {
        return Commands.sequence(elevator.setHeightAndWait(height, 2.0));
    }

    /** Aligns left side, removes algae, and scores. */
    public static Command cleanAndScoreLeft(
            Drivetrain drivetrain,
            Elevator elevator,
            Dealgaefacationinator5000 dealgaefacationinator5000,
            Dispenser dispenser,
            Height dealgaeHeight,
            Height scoreHeight) {
        return Commands.sequence(
                Commands.print("Align left, raise to dealgify at " + dealgaeHeight),
                elevator.setHeight(dealgaeHeight),
                drivetrain.autoAlignToScoringSpot(Side.LEFT).withTimeout(3.0),
                elevator.waitForSetpoint(1.0),
                Commands.print("Cleaning..."),
                dealgaefacationinator5000.clean(),
                Commands.print("Clean! Now, lowering to score height " + scoreHeight),
                elevator.setHeight(scoreHeight),
                Commands.waitSeconds(0.1), // wait a little to let the new setpoint "sink in"
                elevator.waitForSetpoint(1.0),
                Commands.print("Dispensing on cleaned L3"),
                dispenser.dispense().withTimeout(1.0),
                elevator.setHeight(Height.INTAKE));
    }

    public static Command dealgify(
            Drivetrain drivetrain,
            Elevator elevator,
            Dealgaefacationinator5000 dealgaefacationinator5000) {
        return Commands.sequence(
                Commands.print("Align and Raising to dealgify"),
                elevator.setHeight(Height.L3),
                drivetrain.autoAlignToScoringSpot(Side.CENTER).withTimeout(3.0),
                elevator.waitForSetpoint(1.5),
                Commands.print("Cleaning..."),
                dealgaefacationinator5000.clean(),
                Commands.print("Clean!"));
    }

    public static Command scoreSequence(
            Drivetrain drivetrain, Elevator elevator, Dispenser dispenser, Side side) {
        return Commands.sequence(
                Commands.print("ScoreSequence!"),
                drivetrain.autoAlignToScoringSpot(side),
                dispenser.dispense(),
                dispenser.waitForDetectionState(false, 3.0),
                Commands.waitSeconds(2), // TODO: tune this value
                dispenser.stop(),
                elevator.setHeight(Elevator.Height.INTAKE));
    }

    public static Command testingGetReefAlignCommand(
            Drivetrain drivetrain, Elevator elevator, Dispenser dispenser, Height height) {
        return Commands.sequence(
                elevator.setHeight(height),
                // drivetrain.autoAlignToScoringSpot(side),
                dispense(dispenser, elevator, height));
    }
}
