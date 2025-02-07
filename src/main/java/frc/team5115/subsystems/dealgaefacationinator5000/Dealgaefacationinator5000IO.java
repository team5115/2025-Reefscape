package frc.team5115.subsystems.dealgaefacationinator5000;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface Dealgaefacationinator5000IO {

    @AutoLog
    public static class Dealgaefacationinator5000IOInputs {
        double motorVolts;
        double motorVelocityRPM;
        double motorAmps;

        boolean state;
    }

    public default void updateInputs(Dealgaefacationinator5000IOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPneumatic(boolean extend) {}

    public default void setPercent(double percent) {}

    public default void getSparks(ArrayList<SparkMax> sparks) {}
}
