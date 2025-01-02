package frc.team5115.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants;

public class ClimberIOSparkMax implements ClimberIO {
    public final SparkMax leftClimb;
    public final SparkMax rightClimb;
    public final RelativeEncoder leftClimbEncoder;
    public final RelativeEncoder rightClimbEncoder;

    public ClimberIOSparkMax() {
        leftClimb = new SparkMax(Constants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(45).inverted(true);
        leftClimbEncoder = leftClimb.getEncoder();

        rightClimb = new SparkMax(Constants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(45).inverted(false);
        rightClimbEncoder = rightClimb.getEncoder();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.leftAngle = Rotation2d.fromRotations(leftClimbEncoder.getPosition() / 16.0);
        inputs.rightAngle = Rotation2d.fromRotations(rightClimbEncoder.getPosition() / 16.0);
        inputs.leftVelocityRPM = leftClimbEncoder.getVelocity() / 16.0;
        inputs.rightVelocityRPM = rightClimbEncoder.getVelocity() / 16.0;
        inputs.leftCurrentAmps = leftClimb.getOutputCurrent();
        inputs.rightCurrentAmps = rightClimb.getOutputCurrent();
        inputs.leftAppliedVolts = leftClimb.getAppliedOutput();
        inputs.rightAppliedVolts = rightClimb.getAppliedOutput();
    }

    @Override
    public void setLeftPercent(double speed) {
        leftClimb.set(speed);
    }

    @Override
    public void setRightPercent(double speed) {
        rightClimb.set(speed);
    }
}
