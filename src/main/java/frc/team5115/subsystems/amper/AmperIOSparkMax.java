package frc.team5115.subsystems.amper;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;

public class AmperIOSparkMax implements AmperIO {
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;

    public AmperIOSparkMax() {
        motor = new SparkMax(Constants.SNOWBLOWER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        motor.configure(null, null, null);
        encoder.setPositionConversionFactor(360);
        motor.setSmartCurrentLimit(20);
    }

    @Override
    public void updateInputs(AmperIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.position = Rotation2d.fromDegrees(encoder.getPosition());
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
