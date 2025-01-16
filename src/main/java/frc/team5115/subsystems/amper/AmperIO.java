package frc.team5115.subsystems.amper;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AmperIO {
    @AutoLog
    public static class AmperIOInputs {
        public Rotation2d position = new Rotation2d();
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(AmperIOInputs inputs) {}

    /** Run the intake motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run the amper motor at the specified percentage. */
    public default void setPercent(double percent) {}
}
