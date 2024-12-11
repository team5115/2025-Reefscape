package frc.team5115.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        leftMotor = new CANSparkMax(Constants.ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);
        absoluteEncoder = leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(180);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        // leftMotor.setSmartCurrentLimit(40);
        // rightMotor.setSmartCurrentLimit(40);
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
