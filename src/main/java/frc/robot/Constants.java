package frc.robot;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 16;


        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X3_12);


        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.431802);
        public static final double wheelBase = Units.inchesToMeters(19.431802);
        public static final double wheelCircumference = chosenModule.wheelCircumference;


        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;


        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;


        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;


        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 15;
        public static final int angleCurrentThreshold = 25;
        public static final double angleCurrentThresholdTime = 0.05;
        public static final boolean angleEnableCurrentLimit = true;


        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 35;
        public static final double driveCurrentThresholdTime = 0.05;
        public static final boolean driveEnableCurrentLimit = true;


        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.3;
        public static final double closedLoopRamp = 0.2;


        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;


        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;


        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.16;
        public static final double driveKV = 5.647;
        public static final double driveKA = 0.21;


        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10;


        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


         /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(57.92);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-60.9961); //-98.525390625 or //83.232421875 - 180
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
       
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(266.572);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(45.5274);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class SATConstants {
        public static final class SUB{
            public static final double pivot = 79;
            public static final double shooter1 = -35;
            public static final double shooter2 = -35;
        }

        public static final class AMP{
            public static final double pivot = 166.4291992;
            public static final double shooter1 = -15;
            public static final double shooter2 = -15;
        }

        public static final class PODIUM{
            public static final double pivot = 42;
            public static final double shooter1 = -55.0;
            public static final double shooter2 = -35.0;
        }
        
        public static final class WING{
            public static final double pivot = 25;
            public static final double shooter1 = -100.0;
            public static final double shooter2 = -60.0;
        }

        public static final class START{
            public static final double pivot = 0.2;
            public static final double shooter1 = 0.0;
            public static final double shooter2 = 0.0;
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
   
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
   
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final Pose2d tol = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(1));

        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    }

    public static class Vision {
        public static boolean isNoteDetected = false;

        public static boolean visionTurnedOn = false;

        public static boolean isRedAlliance = false;

        public static double gamePieceYawOffset = -56.088;

        public static final Pose2d centerFace =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(218.416),
            Rotation2d.fromDegrees(180));

        /**
         * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when multiple tags are
         * in view in a single camera.
         */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.05;

        public static class gamePieceCameraInfo {
            // public static Transform3d robotToCamera = new Transform3d(12.114, -8.874, 15.266, new Rotation3d(0, 0, 0));
            // public static Transform3d robotToCamera = new Transform3d(10, -3.5, 16.966, new Rotation3d(0, 0, 0));
            public static Transform3d robotToCamera = new Transform3d(10, -3.5, 16.966, new Rotation3d(0, 0, 0));
        }

        public static Pose2d temp = new Pose2d();

        public static class aprilTagBackLeft {
            public static String camera = "AprilTagBackLeft";
            public static Transform3d robotToCamera = new Transform3d(-Units.inchesToMeters(9.647), Units.inchesToMeters(8.923), Units.inchesToMeters(1.7 + 9.217), new Rotation3d(0, 0.785398, -0.785398));
        }
        
        public static class aprilTagFrontRight {
            public static String camera = "AprilTagFrontRight";
            public static Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(5.668), -Units.inchesToMeters(10.631), Units.inchesToMeters(1.7 + 17.470), new Rotation3d(0, -0.785398, -0.785398));
        }

        public static class gamePieceCamera {
            public static String camera = "GamePieceCamera";
            public static Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(5.537), -Units.inchesToMeters(-10.120), Units.inchesToMeters(1.7 + 15.216), new Rotation3d(0, 0, 0));
        }

        public static double getRobotHeading(double gamePieceYaw){
            return ((gamePieceYaw*0.501) + 11.3);
        }

        public static class Sub{
            public static Pose2d bluePose = new Pose2d(new Translation2d(1.38, 5.54), Rotation2d.fromDegrees(0));
            public static Pose2d redPose = convertToRedSide(bluePose);
        }

        public static class SubRight{
            public static Pose2d bluePose = new Pose2d(new Translation2d(0.75, 4.48), Rotation2d.fromDegrees(-60));
            public static Pose2d redPose = convertToRedSide(bluePose);
        }

        public static class Podium{
            public static Pose2d bluePose = new Pose2d(new Translation2d(2.977, 4.082), Rotation2d.fromDegrees(-32.61));
            public static Pose2d redPose = convertToRedSide(bluePose);
        }

        public static double fieldWidth = 16.541;
        public static double fieldHeight = 8.211;

        public static final Pose2d convertToRedSide(Pose2d pose) {
            return new Pose2d(fieldWidth - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180).minus(pose.getRotation()));
        }

        public static Pose2d getPose(String a){
            a = a.toLowerCase();
            if (a == "sub"){
                if (isRedAlliance){
                    return Sub.redPose;
                }
                return Sub.bluePose;
            }
            else if (a == "podium"){
                if (isRedAlliance){
                    return Podium.redPose;
                }
                return Podium.bluePose;
            }
            else if (a == "subright"){
                if (isRedAlliance){
                    return SubRight.redPose;
                }
                return SubRight.bluePose;
            }

            return null;
        }

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static double pivotAngleCalculator(Pose2d currentPose){
            double distance = Math.sqrt(Math.pow(centerFace.getX() - currentPose.getX(), 2)) + Math.pow(centerFace.getY() - currentPose.getY(), 2);

            return distance*1;//math
        }
        
    }

    public static class ArmConstants{
        public static int leaderID = 5;
        public static int followerTalon = 7;
        // public static int armEncoderID = 14;
        public static double armEncoderOffsetRads = 0.0;
        public static boolean leaderInverted = true;

        /* Leader Motor PID Values */
        public static final double leaderKP = 1.5;
        public static final double leaderKI = 0.0;
        public static final double leaderKD = 0.0;
        public static final double leaderKF = 0.0;

        public static int rotorToSensorRatio = 1;
        public static int sensorToMechanismRatio = 1;
        public static Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static double tol = 0.6;
    }

    public static class IntakeConstants{
        public static int kIntakeMotorId = 9;
        public static int kIntakeMotorSpeed = 85;
    }

    public static class IndexConstants{
        public static int kIndexMotorId = 17;
        public static int kIndexMotorSpeed = 85;
    }

    public static class AmperConstants{
        public static int kAmperMotorId = 16;
        public static int kServoID1 = 9;
        public static int kServoID2 = 8;
        public static int kAmperMotorSpeed = 30;
    }

    public static class ShooterConstants{
        public static int kshooterMotorId = 39;
        public static int kshooterMotor1Id = 18;

        public static double tol = 3.5;
    }


/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
    /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
    public static double apply(double xCoordinate) {
    if (shouldFlip()) {
        return Units.inchesToMeters(651.223) - xCoordinate;
    } else {
        return xCoordinate;
    }
    }

    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
        return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
        return translation;
    }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
        return rotation;
    }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
        return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
        return pose;
    }
    }

    public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
        return new Translation3d(
            apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
    } else {
        return translation3d;
    }
    }

    public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    }
}

    public static class ScoringConstants {
        public static ScoringMode currentScoringMode = ScoringMode.PODIUM;
        public enum ScoringMode {
            WING,
            AMP,
            SUBWOOFER,
            PODIUM,
            TRAP,
            AUTOAIM,
            START
        }
    }

    public static boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }

    public static boolean isPoseWithinTol(Pose2d targetPose, Pose2d currentPose, Pose2d tol) {
        return Math.abs(targetPose.getTranslation().getX() - currentPose.getTranslation().getX()) <= tol.getTranslation().getX() &&
               Math.abs(targetPose.getTranslation().getY() - currentPose.getTranslation().getY()) <= tol.getTranslation().getY() &&
               Math.abs(targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()) <= tol.getRotation().getDegrees();
    }
}



