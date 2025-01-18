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
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
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
