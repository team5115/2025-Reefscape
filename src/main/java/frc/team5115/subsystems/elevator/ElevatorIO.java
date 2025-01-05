package frc.team5115.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0;
        public double velocityMetersPerSecond = 0;
        public double currentAmps = 0;
        public double appliedVolts = 0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorVoltage(double volts) {}
}
