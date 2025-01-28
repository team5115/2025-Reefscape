package frc.team5115.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;
import frc.team5115.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController velocityCLC;
    private final DigitalInput backSensor;
    private final DigitalInput firstSensor;
    private final DigitalInput secondSensor;
    private final DigitalInput thirdSensor;

    public ElevatorIOSparkMax() {
        backSensor = new DigitalInput(Constants.BACK_SENSOR_ID);
        firstSensor = new DigitalInput(Constants.FIRST_SENSOR_ID);
        secondSensor = new DigitalInput(Constants.SECOND_SENSOR_ID);
        thirdSensor = new DigitalInput(Constants.THIRD_SENSOR_ID);
        motor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        velocityCLC = motor.getClosedLoopController();
        velocityCLC.setReference(0, ControlType.kVelocity);

        final SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(
                        ElevatorConstants.STALL_CURRENT_AMPS, ElevatorConstants.FREE_CURRENT_AMPS);
        config
                .closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD)
                .velocityFF(1 / ElevatorConstants.KV_NEO_550);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = encoder.getPosition() * ElevatorConstants.METERS_PER_ROTATION;
        inputs.velocityMetersPerSecond =
                encoder.getVelocity() * ElevatorConstants.METERS_PER_ROTATION / 60.0;
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.backCoralDetected = !backSensor.get();
        inputs.firstMagnetDetected = !firstSensor.get();
        inputs.secondMagnetDetected = !secondSensor.get();
        inputs.thirdMagnetDetected = !thirdSensor.get();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setElevatorVelocity(double velocity, double ffVolts) {
        velocityCLC.setReference(
                velocity / ElevatorConstants.METERS_PER_ROTATION * 60,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }
}
