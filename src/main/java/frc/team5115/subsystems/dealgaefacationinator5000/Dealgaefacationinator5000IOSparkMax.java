package frc.team5115.subsystems.dealgaefacationinator5000;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Dealgaefacationinator5000IOSparkMax implements Dealgaefacationinator5000IO {

    Solenoid extender;
    SparkMax motor;

    public Dealgaefacationinator5000IOSparkMax() {
        extender = new Solenoid(PneumaticsModuleType.REVPH, 0); // TODO: set channel
        motor = new SparkMax(0, MotorType.kBrushless);

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(20, 40).idleMode(IdleMode.kCoast);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPneumatic(boolean extend) {
        extender.set(extend);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }
}
