package frc.team5115.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public Rotation2d leftAngle = new Rotation2d();
        public Rotation2d rightAngle = new Rotation2d();
        public double leftVelocityRPM = 0;
        public double rightVelocityRPM = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setLeftPercent(double speed) {}

    public default void setRightPercent(double speed) {}
}
