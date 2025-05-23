package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final class Driving {
            public static final TalonFXConfiguration config = new TalonFXConfiguration();

            static {
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
                config.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;
        
                config.CurrentLimits.SupplyCurrentLimitEnable = true;
                config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.CurrentLimits.kDriving;

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
                double turningFactor = 2 * Math.PI;
                    config
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                    config.absoluteEncoder
                        .inverted(true)
                        .positionConversionFactor(turningFactor)
                        .velocityConversionFactor(turningFactor / 60.0);
                    config.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);
            }
        }
    }

    public static final class Elevator {
        public static final class Master {
            public static final TalonFXConfiguration config = new TalonFXConfiguration();

            static {
                config.Slot0.kS = 0.1;
                config.Slot0.kG = 0;
                config.Slot0.kV = 1;
                config.Slot0.kA = 0.01;
                config.Slot0.kP = 1;
                config.Slot0.kI = 0;
                config.Slot0.kD = 0.1;

                config.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorMotorReduction;
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                config.MotionMagic.MotionMagicCruiseVelocity = 200 / ElevatorConstants.kElevatorMotorReduction;
                config.MotionMagic.MotionMagicAcceleration = 25 / ElevatorConstants.kElevatorMotorReduction;
                config.MotionMagic.MotionMagicJerk = 100 / ElevatorConstants.kElevatorMotorReduction;
            }
        }

        public static final class Slave {
            public static final TalonFXConfiguration config = new TalonFXConfiguration();

            static {
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;                    
            }
        }
    }

    public static final class Coralinator {
        public static final SparkMaxConfig config = new SparkMaxConfig();

        static {
            config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.CoralinatorConstants.CurrentLimits.kCoralinator);
        }
    }
}