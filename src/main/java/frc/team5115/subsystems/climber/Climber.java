package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.commands.DeployClimber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    // Doesn't actually check if the climber has been deployed, just reports if the command has been
    // run since bootup
    private boolean deployed = false;

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/IsDeployed?", deployed);
    }

    public Command deploy() {
        return new DeployClimber(this, +1);
    }

    public Command climbBy(DoubleSupplier speed) {
        return Commands.run(
                () -> {
                    if (isDeployed()) {
                        setPercents(speed.getAsDouble() * 0.3);
                    }
                },
                this);
    }

    public void setPercents(double speed) {
        io.setLeftPercent(speed);
        io.setRightPercent(speed);
    }

    public void stop() {
        setPercents(0);
    }

    public boolean isDeployed() {
        return deployed;
    }

    public void setDeployed() {
        deployed = true;
    }

    public double[] getRotations() {
        return new double[] {inputs.leftAngle.getRotations(), inputs.rightAngle.getRotations()};
    }
}
