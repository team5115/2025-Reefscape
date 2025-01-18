package frc.team5115.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final Elevator elevator;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io, Elevator elevator) {
        this.io = io;
        this.elevator = elevator;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (elevator.checkElevator()) {
            io.setPercent(+1);
        } else {
            io.setPercent(+0);
        }
    }
}
