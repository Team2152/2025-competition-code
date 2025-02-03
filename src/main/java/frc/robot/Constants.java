package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class OIConstants {
        public static final double kDeadband = 0.1;
        public static final double kSpeedLimiterModifier = 6;
    }

    public static final class CANConstants {
        public static final class DrivetrainCANs {
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
    }

    public static final class AutoConstants {
        public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }

    public static final class SwerveConstants {
        public static final class PIDs {
            public static final class Drive {
                public static final double kP = 1;
                public static final double kI = 0;
                public static final double kD = 0;
            }
            public static final class Steer {
                public static final double kP = 1;
                public static final double kI = 0;
                public static final double kD = 0;
            }
        }

        public static final class NeutralModes {
            public static final IdleMode kSteering = IdleMode.kBrake;
            public static final NeutralModeValue kDriving = NeutralModeValue.Coast;
        }

        public static final class CurrentLimits {
            public static final int kDriving = 70;
            public static final int kSteering = 20;
        }

        public static final class Ratios {
            public static final double kDriving = 5.08;
            // public static final double kSteering = 1;
        }

        public static final class Drive {
            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
            public static final double kOpenLoopRampPeriod = 0.25;
        }

        public static final double kWheelDiameter = Units.inchesToMeters(3);
        public static final double kWheelCircumference = (Math.PI * kWheelDiameter);
        public static final double kMaxSpeed = 5.01; // Meters per Second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // Radians
    
        public static final double kLength = Units.inchesToMeters(28);
        public static final double kWidth = Units.inchesToMeters(28);

        public static final double kCenterWheelOffset = Units.inchesToMeters(1.754419);

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d((kLength / 2.0) - kCenterWheelOffset, (kWidth / 2.0) - kCenterWheelOffset),
            new Translation2d((kLength / 2.0) - kCenterWheelOffset, -((kWidth / 2.0) - kCenterWheelOffset)),
            new Translation2d(-((kLength / 2.0) - kCenterWheelOffset), (kWidth / 2.0) - kCenterWheelOffset),
            new Translation2d(-((kLength / 2.0) - kCenterWheelOffset), -((kWidth / 2.0) - kCenterWheelOffset)));

        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    }
}
