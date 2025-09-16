package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.subsystems.drive.Drivetrain;

public class AutoCommands {
    private AutoCommands() {}

    // command for raising while aligning
    public static Command getReefAlignCommand(Drivetrain drivetrain, Side side) {
        return Commands.sequence(drivetrain.autoAlignToScoringSpot(side).withTimeout(2.0));
    }

    /** Aligns left side, removes algae, and scores. */
    public static Command cleanAndScoreLeft(Drivetrain drivetrain) {
        return Commands.sequence(drivetrain.autoAlignToScoringSpot(Side.LEFT).withTimeout(3.0));
    }

    public static Command dealgify(Drivetrain drivetrain) {
        return Commands.sequence(
                Commands.print("Align"), drivetrain.autoAlignToScoringSpot(Side.CENTER).withTimeout(3.0));
    }

    public static Command scoreSequence(Drivetrain drivetrain, Side side) {
        return Commands.sequence(
                Commands.print("ScoreSequence!"), drivetrain.autoAlignToScoringSpot(side));
    }

    public static Command testingGetReefAlignCommand(Drivetrain drivetrain) {
        return Commands.sequence(
                // drivetrain.autoAlignToScoringSpot(side)
                );
    }
}
