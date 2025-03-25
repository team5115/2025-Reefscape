package frc.team5115.subsystems.dealgaefacationinator5000;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

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

    private Command extend() {
        return Commands.runOnce(() -> io.setPneumatic(true), this);
    }

    private Command retract() {
        return Commands.runOnce(() -> io.setPneumatic(false), this);
    }

    private Command spin(double speed) {
        return Commands.runOnce(() -> io.setPercent(speed), this);
    }

    public Command clean() {
        return Commands.sequence(
                extend(),
                spin(0.3),
                waitSeconds(0.2),
                spin(0.9),
                waitSeconds(0.3),
                retract(),
                waitSeconds(0.7),
                spin(0));
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
