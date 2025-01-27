package frc.team5115.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.team5115.Constants.SwerveConstants;

public class ModuleIOSparkMax implements ModuleIO {
    private final SparkMax driveSparkMax;
    private final SparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOSparkMax(int index) {
        int driveId = -1;
        int turnId = -1;

        switch (index) {
            case 0: // Front Left
                driveId = SwerveConstants.FRONT_LEFT_DRIVE_ID;
                turnId = SwerveConstants.FRONT_LEFT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.FRONT_LEFT_ANGULAR_OFFSET;
                break;
            case 1: // Front Right
                driveId = SwerveConstants.FRONT_RIGHT_DRIVE_ID;
                turnId = SwerveConstants.FRONT_RIGHT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.FRONT_RIGHT_ANGULAR_OFFSET;
                break;
            case 2: // Back Left
                driveId = SwerveConstants.BACK_LEFT_DRIVE_ID;
                turnId = SwerveConstants.BACK_LEFT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.BACK_LEFT_ANGULAR_OFFSET;
                break;
            case 3: // Back Right
                driveId = SwerveConstants.BACK_RIGHT_DRIVE_ID;
                turnId = SwerveConstants.BACK_RIGHT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.BACK_RIGHT_ANGULAR_OFFSET;
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        driveSparkMax = new SparkMax(driveId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(turnId, MotorType.kBrushless);

        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getAbsoluteEncoder();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        final SparkMaxConfig turnConfig = new SparkMaxConfig();
        final SparkMaxConfig driveConfig = new SparkMaxConfig();
        final AbsoluteEncoderConfig turnEncoderConfig = new AbsoluteEncoderConfig();
        final EncoderConfig driveEncoderConfig = new EncoderConfig();

        // Old comment:
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        // ! New comment:
        // well it only works when it's NOT inverted so that's what we will do
        turnConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(SwerveConstants.TurningMotorCurrentLimit)
                .voltageCompensation(12.0);

        driveConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(SwerveConstants.DrivingMotorCurrentLimit)
                .voltageCompensation(12.0);

        turnEncoderConfig.averageDepth(2).inverted(false);
        turnConfig.apply(turnEncoderConfig);

        driveEncoderConfig.uvwAverageDepth(2).uvwMeasurementPeriod(10);
        driveConfig.apply(driveEncoderConfig);

        driveSparkMax.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparkMax.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad =
                Units.rotationsToRadians(
                        driveEncoder.getPosition() / SwerveConstants.DrivingMotorReduction);
        inputs.driveVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        driveEncoder.getVelocity() / SwerveConstants.DrivingMotorReduction);
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();
        inputs.turnAbsolutePosition =
                Rotation2d.fromRotations(-turnEncoder.getPosition()).minus(absoluteEncoderOffset);
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnEncoder.getVelocity());
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }
}
