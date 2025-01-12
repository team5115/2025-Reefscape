package frc.team5115.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.team5115.Constants;
import frc.team5115.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public ElevatorIOSparkMax() {
        motor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        final SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = encoder.getPosition() * ElevatorConstants.METERS_PER_ROTATION;
        inputs.velocityMetersPerSecond =
                encoder.getVelocity() * ElevatorConstants.METERS_PER_ROTATION / 60.0;
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
