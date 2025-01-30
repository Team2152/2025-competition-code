package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        private static final double turningFactor = 2 * Math.PI;

        static {
            drivingConfig.Slot0.kS = DriveConstants.kDrivingS;
            drivingConfig.Slot0.kV = DriveConstants.kDrivingV;
            drivingConfig.Slot0.kA = DriveConstants.kDrivingA;

            
            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0);
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0) // Config me!
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }
}