package frc.team5115.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.team5115.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        driveSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 6.75), DCMotor.getNEO(1));
        turnSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.004, 150.0 / 7.0),
                        DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.LOOP_PERIOD_SECS);
        turnSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnAbsolutePosition =
                new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}
