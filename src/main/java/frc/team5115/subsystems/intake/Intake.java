package frc.team5115.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
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

    public Command intake() {
        return setSpeed(+1);
    }

    public Command outtake() {
        return setSpeed(-1);
    }

    public Command stop() {
        return setSpeed(+0);
    }

    public void vomit() {
        io.setPercent(-1);
    }

    public void forceStop() {
        io.setPercent(0);
    }
}
