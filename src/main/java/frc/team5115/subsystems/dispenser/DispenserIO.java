package frc.team5115.subsystems.dispenser;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface DispenserIO {
    @AutoLog
    public static class DispenserIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public boolean frontCoralDetected = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DispenserIOInputs inputs) {}

    /** Run the intake motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run the intake motor at the specified percentage. */
    public default void setPercent(double percent) {}

    /** This should not really be here but it must be */
    public default void getSparks(ArrayList<SparkMax> sparks) {}
}
