package frc.team5115.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.team5115.Constants;

public class IndexerIOSparkMax implements IndexerIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private final DigitalInput sensor;

    public IndexerIOSparkMax() {
        sensor = new DigitalInput(Constants.INDEXER_SENSOR_ID);
        motor = new SparkMax(Constants.INDEXER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(60, 80).idleMode(IdleMode.kCoast);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();

        inputs.coralDetected = !sensor.get();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
