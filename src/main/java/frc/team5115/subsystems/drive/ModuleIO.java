package frc.team5115.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double volts) {}

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(double volts) {}

    /** This should not really be here but it must be */
    public default void getSparks(ArrayList<SparkMax> sparks) {}
}
