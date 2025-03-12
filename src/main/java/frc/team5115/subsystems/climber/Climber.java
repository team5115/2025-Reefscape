package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private boolean extended = false;

    public Climber(ClimberIO io) {
        this.io = io;
        // start retracted
        io.retractSolenoid();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    /** Extends with no check */
    public Command extend() {
        return Commands.runOnce(
                () -> {
                    io.extendSolenoid();
                    extended = true;
                },
                this);
    }

    /** Retracts with no check */
    public Command retract() {
        return Commands.runOnce(
                () -> {
                    io.retractSolenoid();
                    extended = false;
                },
                this);
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this);
    }

    public void stop() {
        io.stopSolenoid();
        extended = false;
    }

    public boolean isCageIntakeDetected() {
        return inputs.cageIntake;
    }

    public Trigger cageDetected() {
        return new Trigger(() -> inputs.cageIntake);
    }

    public Trigger extended() {
        return new Trigger(() -> extended);
    }
}
