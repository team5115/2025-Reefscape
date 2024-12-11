package frc.team5115.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.team5115.Constants;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public IntakeIOSparkMax() {
        motor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        motor.setSmartCurrentLimit(60, 80);
        motor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
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
