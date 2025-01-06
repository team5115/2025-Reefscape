package frc.team5115.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.team5115.Constants;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim;
    private double voltage;

    public ElevatorIOSim() {
        final double minHeight = Elevator.Height.BOTTOM.position;
        final double maxHeight = Elevator.Height.TOP.position;
        // TODO determine gearing, carriage mass, and drum radius
        final double gearing = 10.0; // numbers greater than 1 represent reductions
        final double carriageMassKg = 4.0;
        final double drumRadius = 0.05;
        final double randomStartPosition = Math.random() * (maxHeight - minHeight) + minHeight;
        sim =
                new ElevatorSim(
                        DCMotor.getNEO(1),
                        gearing,
                        carriageMassKg,
                        drumRadius,
                        minHeight,
                        maxHeight,
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
