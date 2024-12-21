package frc.team5115.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;

public class FeederIOSparkMax implements FeederIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final DigitalInput sensor;

    public FeederIOSparkMax() {
        sensor = new DigitalInput(Constants.SHOOTER_SENSOR_ID);

        leftMotor = new SparkMax(Constants.FEEDER_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.FEEDER_RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(false);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.smartCurrentLimit(40);
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(true);
        rightMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.smartCurrentLimit(40);

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.leftVelocityRPM = leftEncoder.getVelocity();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();

        inputs.rightVelocityRPM = rightEncoder.getVelocity();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();

        inputs.noteDetected = !sensor.get();
    }

    @Override
    public void setLeftPercent(double percent) {
        leftMotor.set(percent);
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void setRightPercent(double percent) {
        rightMotor.set(percent);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }
}
