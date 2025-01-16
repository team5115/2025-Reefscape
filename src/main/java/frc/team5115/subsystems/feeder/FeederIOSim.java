package frc.team5115.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.team5115.Constants;

public class FeederIOSim implements FeederIO {

    private final FlywheelSim leftSim;
    private final FlywheelSim rightSim;

    private double leftAppliedVolts;
    private double rightAppliedVolts;

    public FeederIOSim() {
        final DCMotor motor = DCMotor.getNEO(1);
        leftSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.0002, 1.0), motor);
        rightSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.0002, 1.0), motor);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        leftSim.update(Constants.LOOP_PERIOD_SECS);
        rightSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());

        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());

        inputs.noteDetected = false;
    }

    @Override
    public void setRightVoltage(double volts) {
        rightAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setLeftPercent(double percent) {
        leftAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightPercent(double percent) {
        rightAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }
}
