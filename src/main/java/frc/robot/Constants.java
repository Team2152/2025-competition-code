package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class AutoConstants {
    public static RobotConfig kPathplannerConfig;
  }

  public static class DriveConstants {
    public static final double kRobotWidth = Units.inchesToMeters(28);
    public static final double kRobotLength = Units.inchesToMeters(28);

    // Drive PIDs
    public static final double kDrivingS = 0.15;
    public static final double kDrivingA = 25;
    public static final double kDrivingV = 12;

    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    // Turn PIDs
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;

    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    // Heading PIDs
    public static final double kHeadingAlignP = 0.15;
    public static final double kHeadingAlignI = 0;
    public static final double kHeadingAlignD = 0;

    public static final double kHeadingAlignTolerance = 2;

    // Can IDs (Driving)
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;
    // Can IDs (Turning)
    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 9;
    // Pigeon ID
    public static final int kPigeonCanId = 10;

    // Chassis Angle Offsets
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
       public static final double kMaxSpeedMetersPerSecond = 4.0;
       
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d((kRobotWidth-0.0446 / 2) / 2, (kRobotLength-0.0446) / 2),
        new Translation2d((kRobotWidth-0.0446 / 2) / 2, -(kRobotLength-0.0446) / 2),
        new Translation2d(-(kRobotWidth-0.0446 / 2) / 2, (kRobotLength-0.0446) / 2),
        new Translation2d(-(kRobotWidth-0.0446 / 2) / 2, -(kRobotLength-0.0446) / 2));

    public static final double kLimiterModifier = 5.0; // speed / modifier

    public static final boolean kGyroReversed = true;
  }

  public static class SwerveConstants{
    public static final int kDrivingMotorPinionTeeth = 13;

    public static final double kDrivingMotorFreeSpeedRps = FalconMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final boolean kTurningEncoderInverted = true;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  }
    public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.15;

    public static final int kLedPort = 0;
    public static final int kLedLength = 107;

    //public static final Color kLedOrange = new Color(255, 50, 0);
  }
  public static final class FalconMotorConstants {
    public static final double kFreeSpeedRpm = 6380;
  }
}