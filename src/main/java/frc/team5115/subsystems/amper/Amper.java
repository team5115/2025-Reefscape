package frc.team5115.subsystems.amper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Amper extends SubsystemBase {
    private static final double PID_OFFSET = 90.0; // degrees
    private final AmperIO io;
    private final AmperIOInputsAutoLogged inputs = new AmperIOInputsAutoLogged();
    private final PIDController pid;

    public Amper(AmperIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                pid = new PIDController(0.004, 0, 0);
                break;
            case SIM:
                pid = new PIDController(0.005, 0, 0);
                break;
            default:
                pid = new PIDController(0, 0, 0);
                break;
        }

        pid.setTolerance(2);
        pid.setSetpoint(new Rotation2d(0.2).getDegrees() - PID_OFFSET);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Amper", inputs);

        final double pidOutput = pid.calculate(inputs.position.getDegrees() - PID_OFFSET);
        io.setPercent(pidOutput);
        Logger.recordOutput("Amper/Setpoint Degrees", pid.getSetpoint() + PID_OFFSET);
        Logger.recordOutput("Amper/At Setpoint?", pid.atSetpoint());
    }

    public Command spinToAngle(Rotation2d setpoint) {
        return Commands.runOnce(() -> pid.setSetpoint(setpoint.getDegrees() - PID_OFFSET), this)
                .andThen(Commands.waitUntil(() -> pid.atSetpoint()))
                .withTimeout(5);
    }

    public void setSpeed(double percent) {
        io.setPercent(percent);
    }
}
