package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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
        return Commands.runOnce(() -> io.extendSolenoid(), this);
    }

    /** Retracts with no check */
    public Command retract() {
        return Commands.runOnce(() -> io.retractSolenoid(), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> io.stopSolenoid(), this);
    }

    public Command deploy() {
        return Commands.runOnce(
                () -> {
                    if (isCageIntakeDetected()) {
                        io.extendSolenoid();
                    }
                },
                this);
    }

    public void stop() {
        io.stopSolenoid();
    }

    public boolean isCageIntakeDetected() {
        return inputs.cageIntake;
    }
}
