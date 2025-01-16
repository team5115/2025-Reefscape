package frc.team5115.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IndexerIOInputs inputs) {}

    /** Run the intake motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run the intake motor at the specified percentage. */
    public default void setPercent(double percent) {}
}
