package frc.team5115.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.team5115.Constants;
import frc.team5115.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim;
    private double voltage;

    public ElevatorIOSim() {
        final double randomStartPosition =
                Math.random() * (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT)
                        + ElevatorConstants.MIN_HEIGHT;
        sim =
                new ElevatorSim(
                        DCMotor.getNEO(1),
                        ElevatorConstants.GEARING,
                        ElevatorConstants.CARRIAGE_MASS_KG,
                        ElevatorConstants.DRUM_RADIUS,
                        ElevatorConstants.MIN_HEIGHT,
                        ElevatorConstants.MAX_HEIGHT,
                        true,
                        randomStartPosition);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(Constants.LOOP_PERIOD_SECS);

        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.appliedVolts = voltage;
    }

    @Override
    public void setElevatorVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(voltage);
    }
}
