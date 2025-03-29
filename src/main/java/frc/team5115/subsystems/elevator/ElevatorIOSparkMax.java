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
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController velocityCLC;
    private final DigitalInput backSensor;
    private final DigitalInput firstSensor;
    // private final DigitalInput secondSensor;
    // private final DigitalInput thirdSensor;
    // private final DigitalInput fourthSensor;

    public ElevatorIOSparkMax() {
        backSensor = new DigitalInput(Constants.BACK_CORAL_SENSOR);
        firstSensor = new DigitalInput(Constants.ELEVATOR_FIRST_SENSOR_ID);
        // secondSensor = new DigitalInput(Constants.ELEVATOR_SECOND_SENSOR_ID);
        // thirdSensor = new DigitalInput(Constants.ELEVATOR_THIRD_SENSOR_ID);
        // fourthSensor = new DigitalInput(Constants.ELEVATOR_FOURTH_SENSOR_ID);
        motor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        velocityCLC = motor.getClosedLoopController();
        velocityCLC.setReference(0, ControlType.kVelocity);

        final SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(
                        ElevatorConstants.STALL_CURRENT_AMPS, ElevatorConstants.FREE_CURRENT_AMPS);

        config
                .closedLoop
                .p(ElevatorConstants.sparkP)
                .i(ElevatorConstants.sparkI)
                .d(ElevatorConstants.sparkD)
                .velocityFF(1.0 / ElevatorConstants.KV_NEO);

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
        inputs.magnet1detected = !firstSensor.get();
        inputs.magnet2detected = false; // !secondSensor.get();
        inputs.magnet3detected = false; // !thirdSensor.get();
        inputs.magnet4detected = false; // !fourthSensor.get();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        Logger.recordOutput("Elevator/Commanded Voltage", volts);
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

    @Override
    public void getSparks(ArrayList<SparkMax> sparks) {
        sparks.add(motor);
    }
}
