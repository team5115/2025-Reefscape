package frc.team5115.subsystems.amper;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;

public class AmperIOSparkMax implements AmperIO {
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;

    public AmperIOSparkMax() {
        motor = new SparkMax(Constants.SNOWBLOWER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        
        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(20);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AmperIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.position = Rotation2d.fromDegrees(encoder.getPosition() * 360.0);
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
