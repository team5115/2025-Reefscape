package frc.team5115.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.amper.Amper;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.arm.Arm.State;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.feeder.Feeder;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private DriveCommands() {}

    public static Command stow(Arm arm, Intake intake, Feeder feeder, Shooter shooter) {
        return Commands.parallel(arm.stow(), shooter.stop(), feeder.stop(), intake.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command automaticallyPrepareShoot(
            Drivetrain drivetrain, Arm arm, Intake intake, Feeder feeder, Shooter shooter) {
        return drivetrain
                .driveToPosition(new Pose2d()) // ! currently a zero pose, may need to be changed
                .alongWith(prepareShoot(arm, intake, feeder, shooter, State.FAR_SHOT))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command intakeUntilNote(Arm arm, Intake intake, Feeder feeder) {
        return Commands.sequence(
                        intake.intake(),
                        feeder.centerNote(),
                        arm.intake(),
                        feeder.waitForDetectionState(true, 20),
                        Commands.waitSeconds(0.25),
                        intake.stop(),
                        feeder.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command prepareAmp(Arm arm, Amper amper, Intake intake, Feeder feeder) {
        return Commands.sequence(
                        arm.setStateAndWait(State.AMP, 1), amper.spinToAngle(new Rotation2d(3.25)))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command triggerAmp(Arm arm, Amper amper, Intake intake, Feeder feeder) {
        return Commands.sequence(
                        intake.setSpeed(-1),
                        feeder.setSpeeds(-1),
                        feeder.waitForDetectionState(true, 1),
                        feeder.waitForDetectionState(false, 1),
                        Commands.waitSeconds(0.22),
                        intake.stop(),
                        feeder.stop(),
                        Commands.waitSeconds(0.5),
                        amper.spinToAngle(new Rotation2d(0.2)).alongWith(arm.stow()).withTimeout(1.0))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command prepareShoot(
            Arm arm, Intake intake, Feeder feeder, Shooter shooter, Arm.State state) {
        return Commands.parallel(
                        intake.stop(), feeder.stop(), shooter.spinToSpeed(), arm.setStateAndWait(state, 1))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command feed(Intake intake, Feeder feeder) {
        return feed(intake, feeder, 0.5);
    }

    public static Command feed(Intake intake, Feeder feeder, double time) {
        return Commands.sequence(
                feeder.setSpeeds(+1),
                intake.setSpeed(+1),
                Commands.waitSeconds(time),
                feeder.stop(),
                intake.stop());
    }

    public static Command vomit(Intake intake, Feeder feeder, Shooter shooter) {
        return Commands.runOnce(
                () -> {
                    intake.vomit();
                    feeder.vomit();
                    shooter.vomit();
                });
    }

    public static Command forceStop(Intake intake, Feeder feeder, Shooter shooter) {
        return Commands.runOnce(
                () -> {
                    intake.forceStop();
                    feeder.forceStop();
                    shooter.forceStop();
                });
    }

    /**
     * Field or robot relative drive command using two joysticks (controlling linear and angular
     * velocities).
     */
    public static Command joystickDrive(
            Drivetrain drivetrain,
            BooleanSupplier robotRelative,
            BooleanSupplier slowMode,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to ChassisSpeeds & send command
                    final double multiplier = slowMode.getAsBoolean() ? 0.5 : 1.0;
                    final double vx = linearVelocity.getX() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
                    final double vy = linearVelocity.getY() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
                    omega *= SwerveConstants.MAX_ANGULAR_SPEED * multiplier;
                    drivetrain.runVelocity(
                            robotRelative.getAsBoolean()
                                    ? new ChassisSpeeds(vx, vy, omega)
                                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, drivetrain.getGyroRotation()));
                },
                drivetrain);
    }
}
