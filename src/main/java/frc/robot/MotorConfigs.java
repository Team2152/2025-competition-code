package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveConstants;

public class MotorConfigs {
    public static final class MAXSwerveModule {
        public static final class Driving {
            public static final TalonFXConfiguration config = new TalonFXConfiguration();

            static {
                config.MotorOutput.Inverted = SwerveConstants.Drive.kInverted;
                config.MotorOutput.NeutralMode = SwerveConstants.NeutralModes.kDriving;
        
                config.Feedback.SensorToMechanismRatio = SwerveConstants.Ratios.kDriving;
        
                config.CurrentLimits.SupplyCurrentLimitEnable = true;
                config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.CurrentLimits.kDriving;

                config.Slot0.kP = SwerveConstants.PIDs.Drive.kP;
                config.Slot0.kI = SwerveConstants.PIDs.Drive.kI;
                config.Slot0.kD = SwerveConstants.PIDs.Drive.kD;
        
                config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.Drive.kOpenLoopRampPeriod;
                config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.Drive.kOpenLoopRampPeriod;
        
                config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
                config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
            }
        }

        public static final class Turning {
            public static final SparkMaxConfig config = new SparkMaxConfig();

            static {
                double turningFactor = 2 * Math.PI;

                config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(SwerveConstants.CurrentLimits.kSteering);

                config.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor)
                    .velocityConversionFactor(turningFactor / 60.0);

                config.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(SwerveConstants.PIDs.Steer.kP, SwerveConstants.PIDs.Steer.kI, SwerveConstants.PIDs.Steer.kD)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
            }
        }
    }
}
