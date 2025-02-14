package frc.team5115.subsystems.dealgaefacationinator5000;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.team5115.Constants;
import java.util.ArrayList;

public class Dealgaefacationinator5000IOSparkMax implements Dealgaefacationinator5000IO {
    private final DoubleSolenoid extender;
    private final SparkMax motor;
    private boolean state;

    public Dealgaefacationinator5000IOSparkMax(PneumaticHub hub) {
        extender =
                hub.makeDoubleSolenoid(
                        Constants.DEALGAE_FORWARD_CHANNEL, Constants.DEALGAE_REVERSE_CHANNEL);
        motor = new SparkMax(Constants.DEALGAE_MOTOR_ID, MotorType.kBrushless);

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(20, 40).idleMode(IdleMode.kCoast);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(Dealgaefacationinator5000IOInputs inputs) {
        inputs.motorVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorAmps = motor.getOutputCurrent();
        inputs.motorVelocityRPM = motor.getEncoder().getVelocity();

        inputs.state = state;
    }

    @Override
    public void setPneumatic(boolean extend) {
        state = extend;
        extender.set(extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void getSparks(ArrayList<SparkMax> sparks) {
        sparks.add(motor);
    }
}
