package frc.team5115.subsystems.dispenser;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Dispenser extends SubsystemBase {
    private final DispenserIO io;
    private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();

    public Dispenser(DispenserIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Dispenser", inputs);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.frontCoralDetected == state).withTimeout(timeout);
    }

    public Command setSpeed(double percent) {
        return Commands.runOnce(() -> io.setPercent(percent), this);
    }

    public Command dispense() {
        return setSpeed(+0.8);
    }

    public Command dispenseWhileCoral() {
        return Commands.sequence(dispense(), waitForDetectionState(false, 1), stop());
    }

    public Command reverse() {
        return setSpeed(-0.8);
    }

    public Command stop() {
        return setSpeed(0);
    }

    public boolean coralDetected() {
        return inputs.frontCoralDetected;
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }

    public Command slowDispense() {
        return setSpeed(+0.5); // ? TODO determine slow dispense
    }
}
