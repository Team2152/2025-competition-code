package frc.robot.subsystems.DriveTrain;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class SwerveModule {  
    private final TalonFX m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset){
        m_drivingSparkMax = new TalonFX(drivingCANId);
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
        
    }

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