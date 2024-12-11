package frc.team5115.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double leftVelocityRPM = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public double rightVelocityRPM = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;

        public boolean noteDetected = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(FeederIOInputs inputs) {}

    /** Run the left motor at the specified voltage. */
    public default void setLeftVoltage(double volts) {}

    /** Run the left motor at the specified percentage. */
    public default void setLeftPercent(double percent) {}

    /** Run the right motor at the specified voltage. */
    public default void setRightVoltage(double volts) {}

    /** Run the right motor at the specified percentage. */
    public default void setRightPercent(double percent) {}
}
