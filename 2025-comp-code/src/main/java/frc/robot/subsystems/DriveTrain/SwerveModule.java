package frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.PIDConstants;

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
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SwerveConstants.kTurningP)
        .i(SwerveConstants.kTurningI)
        .d(SwerveConstants.kTurningD)
        .outputRange(SwerveConstants.kTurningMinOutput, SwerveConstants.kTurningMaxOutput);
 
        // Set kFF
     configSpark.closedLoop.velocityFF(SwerveConstants.kTurningFF);
    
     configSpark
     .inverted(true)
     .idleMode(IdleMode.kBrake);
 
     configSpark.encoder
     .positionConversionFactor(1000)
     .velocityConversionFactor(1000);

    // Talon config
    SparkMaxConfig configTalon = new SparkMaxConfig();

    // Set PID gains Talon
    configTalon.closedLoop
        .p(SwerveConstants.kDrivingP)
        .i(SwerveConstants.kDrivingI)
        .d(SwerveConstants.kDrivingD)
        .outputRange(SwerveConstants.kDrivingMinOutput, SwerveConstants.kDrivingMaxOutput);
 
        // Set kFF
     //configSpark.closedLoop.velocityFF(1/Kv);
        
        
    _drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    m_drivingEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);


    //m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    //m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    //m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
public class SwerveModule {

    private final TalonFX m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset)
    {
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