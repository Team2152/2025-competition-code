package frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.SwerveConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax; //formerly ka import com.revrobotics.CANSparkMax; //import com.revrobotics.CANSparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkLowLevel.MotorType; //import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig; 
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder; //import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;

//import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final TalonFX m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final PIDController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

   //  new things
   public static final SparkMax.ResetMode safe_params = SparkMax.ResetMode.kResetSafeParameters;
    

  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new TalonFX(drivingCANId);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    //m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingPIDController = new PIDController(SwerveConstants.kDrivingP, SwerveConstants.kDrivingI, SwerveConstants.kDrivingD);
     // Initialize the motor (MAX are setup the same way skibidi? look at this later!!!!!
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    // Set the setpoint of the PID controller in raw position mode
    // Change to Constant later
    double setPoint = 1;
    m_turningPIDController.setReference(setPoint, ControlType.kPosition);

    // Spark config
    SparkMaxConfig configSpark = new SparkMaxConfig();
    //m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.configure(configSpark, safe_params, PersistMode.kPersistParameters);
    // Set PID gains Spark
    configSpark.closedLoop
        .positionWrappingEnabled(true)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SwerveConstants.kTurningP)
        .i(SwerveConstants.kTurningI)
        .d(SwerveConstants.kTurningD)
        .outputRange(SwerveConstants.kTurningMinOutput, SwerveConstants.kTurningMaxOutput);
        double position = m_turningSparkMax.getEncoder().getPosition();
        // Set kFF
     configSpark.closedLoop
     .velocityFF(SwerveConstants.kTurningFF);
    
     configSpark
     .inverted(SwerveConstants.kTurningEncoderInverted)
     .idleMode(SwerveConstants.kTurningMotorIdleMode);
 
     configSpark.encoder
     .positionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor)
     .velocityConversionFactor(SwerveConstants.kTurningEncoderPositionFactor);

    // Talon config
    SparkMaxConfig configTalon = new SparkMaxConfig();

    // Set PID gains Talon
    configTalon.closedLoop
         
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SwerveConstants.kDrivingP)
        .i(SwerveConstants.kDrivingI)
        .d(SwerveConstants.kDrivingD)
        .outputRange(SwerveConstants.kDrivingMinOutput, SwerveConstants.kDrivingMaxOutput);
        // Set kFF
        //configTalon.closedLoop.velocityFF(1/Kv);
        configTalon
        .idleMode(SwerveConstants.kDrivingMotorIdleMode);

        configTalon.encoder
        .positionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);
  }
}
