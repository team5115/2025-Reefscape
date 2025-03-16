package frc.team5115.subsystems.dispenser;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Dispenser extends SubsystemBase {
    public static double l1Speed = 0.45;
    public static double normalSpeed = 0.5;
    public static double altSpeed = 0.6;
    public static double reverseSpeed = -0.25;
    public static double stopSpeed = -0.035;

    private final DispenserIO io;
    private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();
    private final DoubleSupplier speedSupplier;

    public Dispenser(DispenserIO io, DoubleSupplier speedSupplier) {
        this.io = io;
        this.speedSupplier = speedSupplier;
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
        return Commands.run(() -> io.setPercent(speedSupplier.getAsDouble()), this);
    }

    public Command altDispense() {
        return Commands.run(() -> io.setPercent(altSpeed), this);
    }

    public Command dispenseWhileCoral() {
        return Commands.sequence(dispense(), waitForDetectionState(false, 1), stop());
    }

    public Command reverse() {
        return setSpeed(reverseSpeed);
    }

    public Command stop() {
        return setSpeed(stopSpeed);
    }

    public Trigger coralDetected() {
        return new Trigger(() -> inputs.frontCoralDetected);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
