package frc.team5115.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.team5115.Constants;

public class ShooterIOSparkMax implements ShooterIO {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public ShooterIOSparkMax() {
        motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Shooter motor configs
        motor.restoreFactoryDefaults();
        motor.setClosedLoopRampRate(0.1);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
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
