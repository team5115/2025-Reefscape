package frc.team5115.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.team5115.Constants;

public class ShooterIOSparkMax implements ShooterIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public ShooterIOSparkMax() {
        motor = new SparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
