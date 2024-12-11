package frc.team5115.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.team5115.Constants;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim sim;
    private double appliedVolts;

    public ShooterIOSim() {
        sim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.0002);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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
}
