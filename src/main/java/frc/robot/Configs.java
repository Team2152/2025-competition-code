package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final class Driving {
            public static final TalonFXConfiguration config = new TalonFXConfiguration();

            static {
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
                config.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;
        
                config.CurrentLimits.SupplyCurrentLimitEnable = true;
                config.CurrentLimits.SupplyCurrentLimit = 60;

                config.Slot0.kP = 0.04;
                config.Slot0.kI = 0;
                config.Slot0.kD = 0;
        
                config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
                config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        
                config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
                config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
            }
        }

        public static final class Turning {
            public static final SparkMaxConfig config = new SparkMaxConfig();

            static {
                // Use module constants to calculate conversion factors and feed forward gain.
                // double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                //         / ModuleConstants.kDrivingMotorReduction;
                double turningFactor = 2 * Math.PI;
                // double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                // drivingConfig
                //         .idleMode(IdleMode.kBrake)
                //         .smartCurrentLimit(50);
                // drivingConfig.encoder
                //         .positionConversionFactor(drivingFactor) // meters
                //         .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                // drivingConfig.closedLoop
                //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                //         // These are example gains you may need to them for your own robot!
                //         .pid(0.04, 0, 0)
                //         .velocityFF(drivingVelocityFeedForward)
                //         .outputRange(-1, 1);

                    config
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                    config.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                    config.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);
            }
        }
    }
}