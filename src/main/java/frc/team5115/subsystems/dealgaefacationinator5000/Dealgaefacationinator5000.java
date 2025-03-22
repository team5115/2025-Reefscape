package frc.team5115.subsystems.dealgaefacationinator5000;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Dealgaefacationinator5000 extends SubsystemBase {
    private final Dealgaefacationinator5000IO io;
    private final Dealgaefacationinator5000IOInputsAutoLogged inputs =
            new Dealgaefacationinator5000IOInputsAutoLogged();

    public Dealgaefacationinator5000(Dealgaefacationinator5000IO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getName(), inputs);
    }

    public Command extend() {
        return Commands.runOnce(
                () -> {
                    io.setPneumatic(true);
                    io.setPercent(+1.0); // TODO determine clean speed
                },
                this);
    }

    public Command retract() {
        return Commands.runOnce(
                () -> {
                    io.setPneumatic(false);
                    io.setPercent(0);
                },
                this);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
