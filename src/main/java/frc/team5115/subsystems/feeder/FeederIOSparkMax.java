package frc.team5115.subsystems.feeder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;

public class FeederIOSparkMax implements FeederIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final DigitalInput sensor;

    public FeederIOSparkMax() {
        sensor = new DigitalInput(Constants.SHOOTER_SENSOR_ID);

        leftMotor = new CANSparkMax(Constants.FEEDER_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.FEEDER_RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setClosedLoopRampRate(0.1);
        leftMotor.setInverted(false);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setSmartCurrentLimit(40);

        rightMotor.setClosedLoopRampRate(0.1);
        rightMotor.setInverted(true);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setSmartCurrentLimit(40);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
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
