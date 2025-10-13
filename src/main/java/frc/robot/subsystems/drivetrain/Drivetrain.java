// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.utils.AutoAlignPositions;
import frc.lib.utils.LimelightHelpers;
import frc.lib.utils.SwerveWidget;
import frc.lib.utils.Utils;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveConstants.PIDs.Drive;
import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public enum AlignmentStatus {
    LEFT,
    RIGHT,
    CENTER,
    NONE
  }
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      CANConstants.Drivetrain.kFrontLeftDriving,
      CANConstants.Drivetrain.kFrontLeftTurning,
      SwerveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      CANConstants.Drivetrain.kFrontRightDriving,
      CANConstants.Drivetrain.kFrontRightTurning,
      SwerveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      CANConstants.Drivetrain.kBackLeftDriving,
      CANConstants.Drivetrain.kBackLeftTurning,
      SwerveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      CANConstants.Drivetrain.kBackRightDriving,
      CANConstants.Drivetrain.kBackRightTurning,
      SwerveConstants.kBackRightChassisAngularOffset);

  private final PigeonIMU m_gyro = new PigeonIMU(CANConstants.Drivetrain.kPigeon);

  private final Field2d m_field = new Field2d();

  private boolean m_autoAligning = false;
  private Pose2d m_alignmentNode = null;
  
  private boolean m_robotOrriented = false;
  private double m_rotOffset = 0;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_headingController;

  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      SwerveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, 
      new Pose2d()
  );
  
  

    public Drivetrain() {
      try{
        RobotConfig config = RobotConfig.fromGUISettings();
  
        AutoBuilder.configure(
          this::getPose, 
          this::resetPose, 
          this::getSpeeds, 
          this::driveRobotRelative, 
          new PPHolonomicDriveController(
            AutoConstants.kTranslationPID,
            AutoConstants.kRotationPID
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
        );
      }catch(Exception e){
        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      }
  
      m_xController = new PIDController(DrivetrainConstants.PIDs.AutoAlign.kP, DrivetrainConstants.PIDs.AutoAlign.kI, DrivetrainConstants.PIDs.AutoAlign.kD);
      m_yController = new PIDController(DrivetrainConstants.PIDs.AutoAlign.kP, DrivetrainConstants.PIDs.AutoAlign.kI, DrivetrainConstants.PIDs.AutoAlign.kD);
      m_headingController = new PIDController(0.005, 0, 0.0001);
      
      m_xController.setTolerance(.05);
      m_yController.setTolerance(.05);
      m_headingController.setTolerance(2);
      m_headingController.enableContinuousInput(-180, 180);
  
      SmartDashboard.putData("Field Pose", m_field);
      
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

      SmartDashboard.putNumber("RotationOffset", m_rotOffset);
      SmartDashboard.putNumber("Rotation", getHeading());
      
      LimelightHelpers.PoseEstimate limelightMeasurement;
      //LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.MiddleLimelight.kLimelightId, m_rotOffset, m_rotOffset, m_rotOffset, m_rotOffset, m_rotOffset, m_rotOffset);
      LimelightHelpers.SetRobotOrientation(VisionConstants.MiddleLimelight.kLimelightId, m_gyro.getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0);
      if (Utils.isRedAlliance()) {
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed(VisionConstants.MiddleLimelight.kLimelightId);
      } else {
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.MiddleLimelight.kLimelightId);
      }
      
      if (limelightMeasurement != null) {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_odometry.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
        );
      }

      m_field.getObject("AutoAlignTarget").setPose(m_alignmentNode == null ? Pose2d.kZero : m_alignmentNode);
      m_field.setRobotPose(m_odometry.getEstimatedPosition());
    }
  
    public Command setClosestNodeCmd() {
      return runOnce(()->{
        m_alignmentNode = AutoAlignPositions.findClosestPose(getPose(), Utils.isRedAlliance());
      });
    }

    public Command doAutoAlignCmd(Boolean enabled) {
      return runOnce(()->{
        m_autoAligning = enabled;
      });
    }

    public Drivetrain getDrivetrain() {
      return this;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_odometry.getEstimatedPosition();
    }
  
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
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
      boolean useRobotHeading = false;
      if (m_autoAligning && m_alignmentNode != null) {
        useRobotHeading = true;
        // m_alignmentNode = AutoAlignPositions.findClosestPose(getPose(), Utils.isRedAlliance());
        Pose2d currentPose = getPose();

        m_xController.setSetpoint(m_alignmentNode.getX());
        m_yController.setSetpoint(m_alignmentNode.getY());
        m_headingController.setSetpoint(m_alignmentNode.getRotation().getDegrees());

        // xSpeed = m_xController.calculate(rot);
        // ySpeed = m_yController.calculate(rot);
        xSpeed = m_xController.calculate(currentPose.getX());
        ySpeed = m_yController.calculate(currentPose.getY());
        rot = m_headingController.calculate(getPose().getRotation().getDegrees());
      }
      
      // Convert the commanded speeds into the correct units for the drivetrain
      double xSpeedDelivered = xSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedDelivered = ySpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
      double rotDelivered = rot * SwerveConstants.kMaxAngularSpeed;

      if (speedLimiter) {
        xSpeedDelivered = xSpeedDelivered / OIConstants.kLimiterMultiplier;
        ySpeedDelivered = ySpeedDelivered / OIConstants.kLimiterMultiplier;
        rotDelivered = rotDelivered / OIConstants.kLimiterMultiplier;
      }
  
      // Gyro heading should be used for teleop control, robot heading for auto align
      double heading = useRobotHeading ? getPose().getRotation().getDegrees() : getHeading();
      var swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  Rotation2d.fromDegrees(heading))
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
      setStates(swerveModuleStates);
    }
  
      public Command teleopDrive(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier speedLimiter) {
          return run(() -> {
              double xVal = -MathUtil.applyDeadband(x.getAsDouble(), OIConstants.kDriveDeadband);
              double yVal = -MathUtil.applyDeadband(y.getAsDouble(), OIConstants.kDriveDeadband);
              double rotVal = -MathUtil.applyDeadband(rot.getAsDouble(), OIConstants.kDriveDeadband);
  
              boolean speedLimiterVal = speedLimiter.getAsBoolean();
  
              drive(xVal, yVal, rotVal, speedLimiterVal, !m_robotOrriented);
        });
      }

      final SwerveModuleState[] xStates = new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      };
  
    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public Command setX() {
      return run(()-> setStates(xStates));
    }
  
    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
          setStates(desiredStates);
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
  
    public ChassisSpeeds getSpeeds() {
      return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }
  
    public SwerveModuleState[] getModuleStates() {
      return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
      };
    }
  
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
      ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
  
      SwerveModuleState[] targetStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
      setStates(targetStates);
    }
  
    public void setStates(SwerveModuleState[] targetStates) {
      m_frontLeft.setDesiredState(targetStates[0]);
      m_frontRight.setDesiredState(targetStates[1]);
      m_rearLeft.setDesiredState(targetStates[2]);
      m_rearRight.setDesiredState(targetStates[3]);
    }
}