package frc.team5115.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;

public class ArmIOSparkMax implements ArmIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        leftMotor = new SparkMax(Constants.ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = leftMotor.getAbsoluteEncoder();

        final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setArmVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armVelocityRPM = absoluteEncoder.getVelocity() * 60.0;
        if (absoluteEncoder.getPosition() > 160) {
            inputs.armAngle = Rotation2d.fromDegrees(absoluteEncoder.getPosition() - 180.0);
        } else {
            inputs.armAngle = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        }
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    }
}
