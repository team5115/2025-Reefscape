package frc.team5115.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.subsystems.elevator.Elevator;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private static final double INDEXING_SPEED = 0.15;
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

        if (elevator.atIntake()) {
            io.setPercent(INDEXING_SPEED);
        } else {
            io.setPercent(+0);
        }
    }

    public Command setSpeed(double speed) {
        return Commands.runOnce(() -> io.setPercent(speed));
    }

    public Command index() {
        return setSpeed(INDEXING_SPEED);
    }

    public Command stop() {
        return setSpeed(0);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
