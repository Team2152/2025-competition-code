// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
  public static final class LEDConstants {
    public static final class Default {
      public static final Color color = new Color(255, 100, 0);
      public static final boolean blinks = false;
    }

    public static final class Intaking {
      public static final Color color = new Color(0, 255, 0);
      public static final boolean blinks = true;
    }

    public static final class Ready {
      public static final Color color = new Color(0, 255, 0);
      public static final boolean blinks = false;
    }
  }
  public static final class CANConstants {
    public static final class Drivetrain {
      public static final int kFrontLeftDriving = 2;
      public static final int kFrontLeftTurning = 3;

      public static final int kFrontRightDriving = 4;
      public static final int kFrontRightTurning = 5;

      public static final int kBackLeftDriving = 6;
      public static final int kBackLeftTurning = 7;

      public static final int kBackRightDriving = 8;
      public static final int kBackRightTurning = 9;

      public static final int kPigeon = 10;
    }
    public static final class Elevator {
      public static final int kElevatorMaster = 11;
      public static final int kElevatorSlave = 12;
    }
    public static final class Coralinator {
      public static final int kCoralinator = 13;
    }
  }

  public static final class DrivetrainConstants {
    public static final class PIDs {
      public static final class AutoAlign {
        public static final double kP = 0.001;
        public static final double kI = 0;
        public static final double kD = 0;
      }
    }
  }

  public static final class CoralinatorConstants {
    public static final class CurrentLimits {
      public static final int kCoralinator = 30;
    }

    public static final double kIndexingPower = 0.15;
    public static final double kPullbackPower = -0.05;
  }

  public static final class ElevatorConstants {
    public static final class Setpoints {
      public static final double kIntake = 0.5;
      public static final double kL1 = 1;
      public static final double kL2 = 2.65;
      public static final double kL3 = 3.85;
      public static final double kL4 = 7.25;
    }

    public static final class CurrentLimits {
      public static final int kElevator = 40;
    }

    public static final double kElevatorMotorReduction = 9.6429;
    public static final double kElevatorSprocketPitchDiameter = 1.45;
  }

  public static final class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(28 - (1.754419 * 2));
    public static final double kWheelBase = Units.inchesToMeters(28 - (1.754419 * 2));
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;

    public static final class PIDs{
      public static final class Drive {
          public static final double kS = 0.22;
          public static final double kV = 0.05;
          public static final double kA = 0.05;
      }
      public static final class Steer {
          public static final double kP = 1.2;
          public static final double kI = 0;
          public static final double kD = 0;
      }
  }

  public static final class CurrentLimits {
    public static final int kDriving = 55;
    public static final int kSteering = 25;
  }    
}

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 13;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kLimiterMultiplier = 6;
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationPID = new PIDConstants(1, 0, 0);
    public static final PIDConstants kRotationPID = new PIDConstants(1, 0, 0);
  }

  public static final class VisionConstants {
    public static final ArrayList<Integer> kReefTags = new ArrayList<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));

    public static final class LeftLimelight {
      public static final String kLimelightId = "limelight-left";

      public static final double kXTarget = -3.5;
      public static final double kYTarget = -9.8;
    }

    public static final class RightLimelight {
      public static final String kLimelightId = "limelight-right";

      public static final double kXTarget = .9;//1.2;
      public static final double kYTarget = -6;//7;
    }
  }
}
