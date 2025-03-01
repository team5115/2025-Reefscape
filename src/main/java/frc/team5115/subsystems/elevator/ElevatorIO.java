package frc.team5115.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0;
        public double velocityMetersPerSecond = 0;
        public double currentAmps = 0;
        public double appliedVolts = 0;

        public boolean backCoralDetected = false;
        public boolean magnet1detected = false;
        public boolean magnet2detected = false;
        public boolean magnet3detected = false;
        public boolean magnet4detected = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorVoltage(double volts) {}

    // velocity in m/s
    public default void setElevatorVelocity(double velocity, double ffVolts) {}

    /** This should not really be here but it must be */
    public default void getSparks(ArrayList<SparkMax> sparks) {}
}
