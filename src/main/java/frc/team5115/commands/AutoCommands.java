package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.subsystems.dispenser.Dispenser;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.elevator.Elevator;
import frc.team5115.subsystems.elevator.Elevator.Height;

public class AutoCommands {
    private AutoCommands() {}

    // Move elevator to intake position, and then wait until the coral has been fully indexed
    public static Command intakeUntilCoral(Dispenser dispenser, Elevator elevator) {
        // return Commands.print("Intaking").andThen(Commands.waitSeconds(1));
        return Commands.sequence(
                Commands.print("Intaking"),
                elevator.setHeight(Elevator.Height.INTAKE),
                dispenser
                        .waitForDetectionState(true, 5.0)
                        .alongWith(elevator.waitForDetectionState(false, 5.0)));
    }

    // Move elevator to state parameter and then dispense until coral fully exits
    // Then, move back to intake height preemptively
    public static Command dispense(Dispenser dispenser, Elevator elevator, Elevator.Height state) {
        // return Commands.print("Dispensing").andThen(Commands.waitSeconds(1));
        return Commands.sequence(
                Commands.print("Dispensing"),
                elevator.setHeightAndWait(state, 3.0),
                dispenser.dispense(),
                dispenser.waitForDetectionState(false, 0.75),
                Commands.waitSeconds(0.75),
                dispenser.stop(),
                elevator.setHeight(Elevator.Height.INTAKE));
    }

    public static Command getReefAlignCommand(
            Drivetrain drivetrain, Elevator elevator, Dispenser dispenser, Side side, Height height) {
        return Commands.sequence(
                elevator.setHeight(height),
                drivetrain.autoAlignToScoringSpot(side),
                dispense(dispenser, elevator, height));
    }

    public static Command raiseElevator(Elevator elevator, Height height) {
        return Commands.sequence(elevator.setHeightAndWait(height, 2.0));
    }

    public static Command scoreSequence(
            Drivetrain drivetrain, Elevator elevator, Dispenser dispenser, Side side) {
        return Commands.sequence(
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
