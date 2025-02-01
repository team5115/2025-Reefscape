package frc.team5115.subsystems.dispenser;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;
import java.util.ArrayList;

public class DispenserIOSparkMax implements DispenserIO {
    private final SparkMax motor;

    private final DigitalInput frontSensor;

    public DispenserIOSparkMax() {
        frontSensor = new DigitalInput(Constants.DISPENSER_SENSOR_ID);
        motor = new SparkMax(Constants.DISPENSER_MOTOR_ID, MotorType.kBrushed);

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40, 40).idleMode(IdleMode.kCoast);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(DispenserIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();

        inputs.frontCoralDetected = !frontSensor.get();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void getSparks(ArrayList<SparkMax> sparks) {
        sparks.add(motor);
    }
}
