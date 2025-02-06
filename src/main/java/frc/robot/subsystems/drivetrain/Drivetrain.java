// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.utils.SwerveWidget;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      CANConstants.DrivetrainCANs.kFrontLeftDriving,
      CANConstants.DrivetrainCANs.kFrontLeftTurning,
      SwerveConstants.kFrontLeftChassisAngularOffset,
      false);

  private final SwerveModule m_frontRight = new SwerveModule(
      CANConstants.DrivetrainCANs.kFrontRightDriving,
      CANConstants.DrivetrainCANs.kFrontRightTurning,
      SwerveConstants.kFrontRightChassisAngularOffset,
      false);

  private final SwerveModule m_rearLeft = new SwerveModule(
      CANConstants.DrivetrainCANs.kBackLeftDriving,
      CANConstants.DrivetrainCANs.kBackLeftTurning,
      SwerveConstants.kBackLeftChassisAngularOffset,
      false);

  private final SwerveModule m_rearRight = new SwerveModule(
      CANConstants.DrivetrainCANs.kBackRightDriving,
      CANConstants.DrivetrainCANs.kBackRightTurning,
      SwerveConstants.kBackRightChassisAngularOffset,
      false);

  private final PigeonIMU m_gyro = new PigeonIMU(CANConstants.DrivetrainCANs.kPigeon);

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      SwerveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  public Drivetrain() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SwerveWidget.sendWidget(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight, m_gyro);
  }

  @Override
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean speedLimiter, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * SwerveConstants.kMaxAngularSpeed;

    if (speedLimiter) {
      xSpeedDelivered = xSpeedDelivered / OIConstants.kLimiterMultiplier;
      ySpeedDelivered = ySpeedDelivered / OIConstants.kLimiterMultiplier;
      rotDelivered = rotDelivered / OIConstants.kLimiterMultiplier;
    }

    var swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

    public Command teleopDrive(
    DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier speedLimiter) {
        return run(() -> {
            double xVal = -MathUtil.applyDeadband(x.getAsDouble(), OIConstants.kDriveDeadband);
            double yVal = -MathUtil.applyDeadband(y.getAsDouble(), OIConstants.kDriveDeadband);
            double rotVal = -MathUtil.applyDeadband(rot.getAsDouble(), OIConstants.kDriveDeadband);

            boolean speedLimiterVal = speedLimiter.getAsBoolean();

            drive(xVal, yVal, rotVal, speedLimiterVal, true);
      });
    }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. 
 * @return */
  public Command zeroHeading() {
    return runOnce( ()-> m_gyro.setFusedHeading(0));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getFusedHeading()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getFusedHeading() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}