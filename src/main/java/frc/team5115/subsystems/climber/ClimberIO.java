package frc.team5115.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean cageIntake = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void extendSolenoid() {}

    public default void retractSolenoid() {}

    public default void stopSolenoid() {}

    public default void toggleBlock() {}
}
