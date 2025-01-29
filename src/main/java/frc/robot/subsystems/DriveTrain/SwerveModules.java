package frc.robot.subsystems.DriveTrain;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class SwerveModules {

    public void configureMotors() {
        // Initialize SparkMax motor controller
        SparkMax max = new SparkMax(1, MotorType.kBrushless);

        // Create a configuration object
        SparkMaxConfig config = new SparkMaxConfig();

    config
    .inverted(true)
    .idleMode(IdleMode.kBrake);

    config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);

    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);

    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}