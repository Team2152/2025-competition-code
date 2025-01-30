// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private final TalonFX m_drivingMotor;
  private final SparkMax m_turningMotor;

  private final AbsoluteEncoder m_turningEncoder;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(DriveConstants.kDrivingS, DriveConstants.kDrivingV, DriveConstants.kDrivingA);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingMotor = new TalonFX(drivingCANId);
    m_turningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();
    m_turningClosedLoopController = m_turningMotor.getClosedLoopController();
    m_turningMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_drivingMotor.getConfigurator().apply(Configs.MAXSwerveModule.drivingConfig);
    m_drivingMotor.getConfigurator().setPosition(0);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

    m_drivingMotor.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingMotor.getVelocity().getValueAsDouble(),
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
        m_drivingMotor.get(),
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
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    driveVelocity.Velocity = (desiredState.speedMetersPerSecond / (Math.PI * 3) * 60);
    driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
    m_drivingMotor.setControl(driveVelocity);

    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingMotor.setPosition(0);
  }
}