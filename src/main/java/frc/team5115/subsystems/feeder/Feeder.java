package frc.team5115.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public Feeder(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.noteDetected == state).withTimeout(timeout);
    }

    public Command stop() {
        return setSpeeds(+0);
    }

    public Command centerNote() {
        return setSpeeds(+0.08);
    }

    public Command setSpeeds(double percent) {
        return setSpeeds(percent, percent);
    }

    public Command setSpeeds(double leftPercent, double rightPercent) {
        return Commands.runOnce(
                () -> {
                    io.setLeftPercent(leftPercent);
                    io.setRightPercent(rightPercent);
                },
                this);
    }

    public void vomit() {
        io.setLeftPercent(-0.9);
        io.setRightPercent(-0.9);
    }

    public void forceStop() {
        io.setLeftPercent(0);
        io.setRightPercent(0);
    }

    public boolean noteDetected() {
        return inputs.noteDetected;
    }
}
