package frc.team5115.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command setSpeed(double percent) {
        return Commands.runOnce(() -> io.setPercent(percent), this);
    }

    public Command index() {
        return setSpeed(+1);
    }

    public Command stop() {
        return setSpeed(+0);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.coralDetected == state).withTimeout(timeout);
    }
}
