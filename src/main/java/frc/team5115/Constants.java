package frc.team5115;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team5115.subsystems.elevator.Elevator;
import java.util.ArrayList;
import java.util.List;

public final class Constants {
    private static final boolean isReplay = false;
    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final byte PNEUMATIC_HUB_ID = 2;
    public static final byte CLIMB_FORWARD_CHANNEL = 0;
    public static final byte CLIMB_REVERSE_CHANNEL = 1;
    public static final byte DEALGAE_FORWARD_CHANNEL = 2;
    public static final byte DEALGAE_REVERSE_CHANNEL = 3;

    public static final byte ELEVATOR_MOTOR_ID = 11;
    public static final byte INTAKE_MOTOR_ID = 12;
    public static final byte DISPENSER_MOTOR_ID = 13;
    public static final byte DEALGAE_MOTOR_ID = 14;

    public static final byte CLIMB_INAKE_SENSOR = 2;
    public static final byte BACK_CORAL_SENSOR = 0;
    public static final byte FRONT_CORAL_SENSOR = 3;

    public static final byte ELEVATOR_FIRST_SENSOR_ID = 1;
    public static final byte ELEVATOR_SECOND_SENSOR_ID = 4;
    public static final byte ELEVATOR_THIRD_SENSOR_ID = 9;
    // public static final byte ELEVATOR_FOURTH_SENSOR_ID = 6;

    public static final byte BLOCK_ACTUATOR_ID = 9;

    public static final byte LED_STRIP_PWM_ID = 0;

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static class SwerveConstants {
        public static final byte FRONT_LEFT_DRIVE_ID = 6;
        public static final byte FRONT_RIGHT_DRIVE_ID = 4;
        public static final byte BACK_LEFT_DRIVE_ID = 10;
        public static final byte BACK_RIGHT_DRIVE_ID = 8;

        public static final byte FRONT_LEFT_TURN_ID = 5;
        public static final byte FRONT_RIGHT_TURN_ID = 3;
        public static final byte BACK_LEFT_TURN_ID = 9;
        public static final byte BACK_RIGHT_TURN_ID = 7;

        private static RobotConfig ROBOT_CONFIG = null;

        public static RobotConfig getRobotConfig() {
            if (ROBOT_CONFIG == null) {
                try {
                    ROBOT_CONFIG = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return ROBOT_CONFIG;
        }

        public static final double MAX_LINEAR_SPEED = 5; // meters per second

        private static final double TRACK_WIDTH = Units.inchesToMeters(26.25);
        public static final double TRACK_WIDTH_X = TRACK_WIDTH;
        public static final double TRACK_WIDTH_Y = TRACK_WIDTH;
        public static final double DRIVE_BASE_RADIUS =
                Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        // Required for inverse kinematics. +x is forward, +y is left
        // The module order, as with everywhere else, is FL, FR, BL, BR
        public static final Translation2d[] MODULE_TRANSLATIONS =
                new Translation2d[] {
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / -2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / -2.0)
                };

        public static final Rotation2d FRONT_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(270);
        public static final Rotation2d FRONT_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BACK_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(180);
        public static final Rotation2d BACK_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(90);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear
        // 15 teeth on the bevel pinion, 13 teeth on the driving motor
        public static final double DrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);

        public static final int DrivingMotorAutoCurrentLimit = 60; // amp
        public static final int DrivingMotorTeleopCurrentLimit = 50; // amps, lower than in auto
        public static final int TurningMotorCurrentLimit = 20; // amps
    }

    public static class ElevatorConstants {
        // Heights and radius in meters
        public static final double MAX_HEIGHT = Elevator.Height.L4.position;
        public static final double MIN_HEIGHT = Elevator.Height.MINIMUM.position;
        public static final double DRUM_RADIUS = 0.05;
        public static final double CARRIAGE_MASS_KG = 9.072; // 20 lbs
        public static final double GEARING = 12.0; // numbers greater than 1 represent reductions

        // Below conversion factor found empirically
        public static final double METERS_PER_ROTATION =
                0.007778940626786609 / 52.0 * 57.0 * 20.0 / GEARING * 63.0 / 62.625;

        public static final int STALL_CURRENT_AMPS = 40;
        public static final int FREE_CURRENT_AMPS = 40;
        // Kv values for neo and neo 550
        // Kf for closed loop velocity control is 1 / kv
        public static final double KV_NEO = 473;
        public static final double KV_NEO_550 = 917;

        public static final double sparkP = 0.00001;
        public static final double sparkI = 0;
        public static final double sparkD = 0;

        public static final double SLOW_PID_HEIGHT_METERS = 0.01 * 6.0;

        public static final double FIRST_MAGNET_HEIGHT = 0;
        public static final double SECOND_MAGNET_HEIGHT = 0.1637495863622731; // meters
        public static final double THIRD_MAGNET_HEIGHT = 0.6174441447379277; // meters

        public static final double MAX_VEL = 10.0; // m/s
        public static final double MAX_ACCEL = 20.0; // m/s^2
        public static final double KP = 3.0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0.38176; // volts
        public static final double KG = 0.33756; // volts
        public static final double KV = 9.0313; // volts / m/s
        public static final double KA = 0.44379; // volts / m/s^2

        public static final double SLOW_CONSTANT = 0.5;
    }

    public static class AutoConstants {
        private static final List<Pose2d> leftScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> rightScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> centerScoringPoses = new ArrayList<Pose2d>();

        public enum Side {
            LEFT(leftScoringPoses),
            RIGHT(rightScoringPoses),
            CENTER(centerScoringPoses);

            public final List<Pose2d> poses;

            Side(final List<Pose2d> poses) {
                this.poses = poses;
            }
        }

        private static final double forwardOffset = 0.46; // distance from the april tag
        private static final Transform2d transformLeft =
                new Transform2d(new Translation2d(forwardOffset, -0.36), Rotation2d.k180deg);
        private static final Transform2d transformRight =
                new Transform2d(new Translation2d(forwardOffset, +0.0), Rotation2d.k180deg);
        private static final Transform2d transformCenter =
                new Transform2d(new Translation2d(forwardOffset, -0.25), Rotation2d.k180deg);

        public static Pose2d getNearestScoringSpot(final Pose2d robot, final Side side) {
            double shortestDistance = Double.MAX_VALUE;
            Pose2d closestPose = null;
            for (final Pose2d pose : side.poses) {
                final double distance = pose.getTranslation().getDistance(robot.getTranslation());
                if (distance < shortestDistance) {
                    shortestDistance = distance;
                    closestPose = pose;
                }
            }
            return closestPose;
        }

        public static void precomputeAlignmentPoses() {
            for (final AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
                if (((tag.ID >= 6 && tag.ID <= 11) || (tag.ID >= 17 && tag.ID <= 22))) {
                    final Pose2d tagPose = tag.pose.toPose2d();
                    leftScoringPoses.add(tagPose.transformBy(transformLeft));
                    rightScoringPoses.add(tagPose.transformBy(transformRight));
                    centerScoringPoses.add(tagPose.transformBy(transformCenter));
                }
            }
        }

        // Calculated using AndyMark april tag json combined with field cad
        // 0.818973 is the distance from the center of the reef to the center of an edge
        private static final Translation2d blueReefCenter =
                new Translation2d(5.321046 - 0.818973, 4.02082);
        private static final Translation2d redReefCenter =
                new Translation2d(12.227306 + 0.818973, 4.02082);

        public static double getReefX(boolean isRedAlliance) {
            return isRedAlliance ? redReefCenter.getX() : blueReefCenter.getX();
        }

        public static double getReefY(boolean isRedAlliance) {
            return isRedAlliance ? redReefCenter.getY() : blueReefCenter.getY();
        }
    }

    public static class VisionConstants {

        // private static AprilTagFieldLayout loadReefOnlyFieldLayout() {
        //     try {
        //         // throw new IOException();
        //         return new AprilTagFieldLayout(
        //                 Filesystem.getDeployDirectory().getAbsolutePath()
        //                         + File.separatorChar
        //                         + "reef_only.json");
        //     } catch (IOException e) {
        //         e.printStackTrace();
        //         return loadFullField();
        //     }
        // }

        private static AprilTagFieldLayout loadFullField() {
            return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        }

        public static final AprilTagFieldLayout FIELD_LAYOUT = loadFullField();

        // Camera sim values
        public static final int WIDTH_PX = 1280;
        public static final int HEIGHT_PX = 720;
        public static final double DIAG_FOV_DEGREES = 90;
        public static final double AVG_ERR_PX = 1;
        public static final double STD_DEV_ERR_PX = 0;
        public static final double FPS = 30;
        public static final double AVG_LATENCY_MS = 30;
        public static final double STD_DEV_LATENCY_MS = 10;

        // Pose filtering values
        public static final double distanceThreshold = 1.5; // meters
        public static final double angleThreshold = 10.0; // degrees
        public static final double zTranslationThreshold = 0.15; // meters
        public static final double ambiguityThreshold = 0.5;
        // every tag beyond seeing two tags gives us an extra meter of trusted distance
        public static final double multiTagDistanceFactor = 1.0;
    }
}
