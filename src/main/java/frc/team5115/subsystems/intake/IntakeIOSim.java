package frc.team5115.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.team5115.Constants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim sim;
    private double appliedVolts;

    public IntakeIOSim() {
        final DCMotor motor = DCMotor.getNEO(1);
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.0002, 1.0), motor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        sim.update(Constants.LOOP_PERIOD_SECS);
        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setPercent(double percent) {
        appliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        sim.setInputVoltage(appliedVolts);
    }
}
