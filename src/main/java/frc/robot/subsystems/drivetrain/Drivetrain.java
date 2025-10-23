// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.utils.LimelightHelpers;
import frc.lib.utils.ReefProximity;
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
import edu.wpi.first.math.util.Units.*;

public class Drivetrain extends SubsystemBase {
  public enum AlignmentStatus {
    LEFT,
    RIGHT,
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

  private AlignmentStatus m_alignmentStatus = AlignmentStatus.NONE;
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
  
  public AprilTagFieldLayout aprilTagFieldLayout;
    private static final List<Integer> redTags = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    private static final List<Integer> blueTags = List.of(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);
    public static final List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    public static final List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);
    public static final List<Integer> allReefTags = new ArrayList<>();

    private final Distance reefBackDistance = Units.Inches.of(13);
    private final Distance reefSideDistance = Units.Inches.of(13).div(2);

    private final Transform2d leftReefTransform = new Transform2d(reefBackDistance.in(Units.Meters), -reefSideDistance.in(Units.Meters), Rotation2d.k180deg);
    private final Transform2d rightReefTransform = new Transform2d(reefBackDistance.in(Units.Meters), reefSideDistance.in(Units.Meters), Rotation2d.k180deg);

    public final HashMap<Integer, Pose2d> tagPoses2d = new HashMap<>();
    public final HashMap<Integer, Pose2d> leftReefHashMap = new HashMap<>();
    public final HashMap<Integer, Pose2d> rightReefHashMap = new HashMap<>();

    public final ReefProximity reefProximity;

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
  
      m_xController = new PIDController(DrivetrainConstants.PIDs.AutoAlignPos.kP, DrivetrainConstants.PIDs.AutoAlignPos.kI, DrivetrainConstants.PIDs.AutoAlignPos.kD);
      m_yController = new PIDController(DrivetrainConstants.PIDs.AutoAlignPos.kP, DrivetrainConstants.PIDs.AutoAlignPos.kI, DrivetrainConstants.PIDs.AutoAlignPos.kD);
      m_headingController = new PIDController(DrivetrainConstants.PIDs.AutoAlignRot.kP,DrivetrainConstants.PIDs.AutoAlignRot.kI,DrivetrainConstants.PIDs.AutoAlignRot.kD);
      
      m_xController.setTolerance(.0127);
      m_yController.setTolerance(.0127);
      m_headingController.setTolerance(2);
      m_headingController.enableContinuousInput(-180, 180);
  
      SmartDashboard.putData("Field Pose", m_field);
      
      HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  
      SwerveWidget.sendWidget(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight, m_gyro);

      //LimelightHelpers.SetRobotOrientation(VisionConstants.MiddleLimelight.kLimelightId, m_gyro.getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0);

      try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
      }

      allReefTags.addAll(redReefTags);
      allReefTags.addAll(blueReefTags);

      for (var allianceTags : List.of(redTags, blueTags)) {
            for (int tagID : allianceTags) {
                Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(tagID);
                if (pose3d.isPresent()) {
                    tagPoses2d.put(tagID, pose3d.get().toPose2d());
                }
            }
      }

      for (int i : redReefTags) {
        leftReefHashMap.put(i, calculateReefPose(i, true));
        rightReefHashMap.put(i, calculateReefPose(i, false));
      }
      for (int i : blueReefTags) {
          leftReefHashMap.put(i, calculateReefPose(i, true));
          rightReefHashMap.put(i, calculateReefPose(i, false));
      }
      reefProximity = new ReefProximity(leftReefHashMap, rightReefHashMap);
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
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.MiddleLimelight.kLimelightId);
      
      if (limelightMeasurement != null && limelightMeasurement.tagCount > 0) {
        m_odometry.setVisionMeasurementStdDevs(MatBuilder.fill(Nat.N3(), Nat.N1(), 0.025, 0.025, 0.025));
        m_odometry.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
        );
      }

      // if (m_autoAligning) {
      //   Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(getPose(), allReefTags);
      //       if (closestTagAndPose == null) {
      //           m_alignmentNode = Pose2d.kZero;
      //       } else {
      //           m_alignmentNode = closestTagAndPose.getValue();
      //       }
      //     }

      m_field.getObject("AutoAlignTarget").setPose(m_alignmentNode == null ? Pose2d.kZero : m_alignmentNode);
      m_field.setRobotPose(m_odometry.getEstimatedPosition());

      m_autoAligning = m_alignmentStatus != AlignmentStatus.NONE;
    }

    public Command driveCmd() {
      return run(() -> {drive(0, 0, 0, false, true);});
    }

    public boolean autoAlignNodeDistance(double meters) {
      return getPose().getTranslation().getDistance(m_alignmentNode.getTranslation()) <= meters;
    }

    public Command doAutoAlignCmd(AlignmentStatus status) {
      return runOnce(()->{
        m_alignmentStatus = status;

        Entry<Integer, Pose2d> closestTagAndPose = reefProximity.closestReefPose(getPose(), allReefTags);
        if (closestTagAndPose == null) {
            m_alignmentNode = Pose2d.kZero;
        } else {
            m_alignmentNode = reefProximity.getReefPose(closestTagAndPose.getKey(), m_alignmentStatus == AlignmentStatus.LEFT);
        }
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

      if (speedLimiter || m_autoAligning) {
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


    // Vision
    private Pose2d getReefTagPose(int tagID) {
      Pose2d pose = tagPoses2d.get(tagID);
      if (pose == null) return null;

      // if (TAG_OFFSETS.containsKey(tagID)) {
      //     pose = pose.transformBy(new Transform2d(0, TAG_OFFSETS.get(tagID).in(Meters), Rotation2d.kZero));
      // }
      return pose;
    }

    /**
     * Generates the scoring pose of the robot relative to a reef AprilTag. This is used to pre-calculate and store all
     * positions to prevent duplicate object creation. To access these pre-calculated poses, use {@link #getReefPose(int, boolean)}.
     */
    private Pose2d calculateReefPose(int tagID, boolean left) {
      Pose2d pose = getReefTagPose(tagID);
      if (pose == null) return null;

      return pose.transformBy(left ? leftReefTransform : rightReefTransform);
    }

    public int getReefTagFromPose(Pose2d pose) {
      for (Entry<Integer, Pose2d> entry : leftReefHashMap.entrySet()) {
          if (entry.getValue().equals(pose)) {
              return entry.getKey();
          }
      }
      for (Entry<Integer, Pose2d> entry : rightReefHashMap.entrySet()) {
          if (entry.getValue().equals(pose)) {
              return entry.getKey();
          }
      }
      return -1;
    }
  }