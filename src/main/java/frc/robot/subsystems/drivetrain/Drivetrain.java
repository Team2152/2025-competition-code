package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveWidget;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANConstants.DrivetrainCANs;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
    private final PigeonIMU m_gyro;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final Field2d m_field = new Field2d();

    public Drivetrain() {
        m_gyro = new PigeonIMU(DrivetrainCANs.kPigeon);
        
        m_frontLeft = new SwerveModule(DrivetrainCANs.kFrontLeftDriving, DrivetrainCANs.kFrontLeftTurning, SwerveConstants.kFrontLeftChassisAngularOffset);
        m_frontRight = new SwerveModule(DrivetrainCANs.kFrontRightDriving, DrivetrainCANs.kFrontRightTurning, SwerveConstants.kFrontRightChassisAngularOffset);
        m_backLeft = new SwerveModule(DrivetrainCANs.kBackLeftDriving, DrivetrainCANs.kBackLeftTurning, SwerveConstants.kBackLeftChassisAngularOffset);
        m_backRight = new SwerveModule(DrivetrainCANs.kBackRightDriving, DrivetrainCANs.kBackRightTurning, SwerveConstants.kBackRightChassisAngularOffset);

        m_poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.kSwerveKinematics,
            getHeading(), getPositions(), new Pose2d()
        );

        SwerveWidget.sendWidget(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro);

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
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(getHeading(), getPositions());

        m_field.setRobotPose(getPose());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean limiterEnabled) {
        double xSpeedCommanded = xSpeed;
        double ySpeedCommanded = ySpeed;
        double rotDelivered = rot;
    
        xSpeedCommanded *= SwerveConstants.kMaxSpeed;
        ySpeedCommanded *= SwerveConstants.kMaxSpeed;
        rotDelivered *= SwerveConstants.kMaxAngularSpeed;
    
        if (limiterEnabled) {
            xSpeedCommanded /= OIConstants.kSpeedLimiterModifier;
            ySpeedCommanded /= OIConstants.kSpeedLimiterModifier;
            rotDelivered /= OIConstants.kSpeedLimiterModifier;
        }

        var swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedCommanded, ySpeedCommanded, rotDelivered, Rotation2d.fromDegrees(m_gyro.getFusedHeading())));
        //SwerveDriveKinematics.desaturateWheelSpeeds(
        //        swerveModuleStates, SwerveConstants.kMaxSpeed);

        setStates(swerveModuleStates);
      }

    public Command teleopDrive(
    DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier speedLimiter) {
        return run(() -> {
            double xVal = -MathUtil.applyDeadband(x.getAsDouble(), OIConstants.kDeadband);
            double yVal = -MathUtil.applyDeadband(y.getAsDouble(), OIConstants.kDeadband);
            double rotVal = -MathUtil.applyDeadband(rot.getAsDouble(), OIConstants.kDeadband);

              boolean speedLimiterVal = speedLimiter.getAsBoolean();

              drive(xVal, yVal, rotVal, speedLimiterVal);
      });
    }

    /**
    * Sets the wheels into an X formation to prevent movement.
    */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Returns the current Pose2d.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the Pose Estimator Pose2d.
     * @param pose New Pose2d
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }

    /**
     * Returns the current gyro heading.
     */
    public Rotation2d getHeading() {
        double degrees = m_gyro.getFusedHeading();
        double radians = Conversions.degreesToRadians(degrees);
        return new Rotation2d(radians);
    }

    /**
     * Sets the gyro heading to a new value.
     */
    public Command zeroGyro(double heading) {
        return runOnce(()->m_gyro.setFusedHeading(heading));
    }

    /*
     * Returns a ChassisSpeeds of the current modules.
     */
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /*
     * Sets all the swerve modules to target states.
     */
    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.kMaxSpeed);
    
        m_frontLeft.setDesiredState(targetStates[0]);
        m_frontRight.setDesiredState(targetStates[1]);
        m_backLeft.setDesiredState(targetStates[2]);
        m_backRight.setDesiredState(targetStates[3]);
    }

    /*
     * Drives with a robot relavtive ChassisSpeeds
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    /*
     * Returns all of the swerve module states.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    /*
     * Returns all of the swerve module positions
     */
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }
}
