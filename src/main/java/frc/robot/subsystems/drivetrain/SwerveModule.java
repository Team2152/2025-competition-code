package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.robot.MotorConfigs;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private double angleOffset;

    private int m_driveID;
    private int m_angleID;

    private SparkMax m_angleMotor;
    private TalonFX m_driveMotor;

    private AbsoluteEncoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.PIDs.Drive.kP, SwerveConstants.PIDs.Drive.kI, SwerveConstants.PIDs.Drive.kD);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    private final SparkClosedLoopController m_turningClosedLoopController;

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset){
        this.angleOffset = chassisAngularOffset;

        this.m_driveID = drivingCANId;
        this.m_angleID = turningCANId;

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(m_driveID);
        m_driveMotor.getConfigurator().apply(MotorConfigs.MAXSwerveModule.Driving.config);
        m_driveMotor.getConfigurator().setPosition(0.0);

        /* Angle Motor Config */
        m_angleMotor = new SparkMax(m_angleID, MotorType.kBrushless);
        m_angleMotor.configure(MotorConfigs.MAXSwerveModule.Turning.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_turningClosedLoopController = m_angleMotor.getClosedLoopController();

        angleEncoder = m_angleMotor.getAbsoluteEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angleOffset));
    
        correctedDesiredState.optimize(new Rotation2d(angleEncoder.getPosition()));

        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
        driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, SwerveConstants.kWheelCircumference);
        driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        m_driveMotor.setControl(driveVelocity);

        System.out.println("SPEED :  " + correctedDesiredState.speedMetersPerSecond);
        System.out.println("ANGLE :  " + correctedDesiredState.angle.getRadians());
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getPosition());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(m_driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.kWheelCircumference), 
            Rotation2d.fromRotations(angleEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(m_driveMotor.getPosition().getValueAsDouble(), SwerveConstants.kWheelCircumference), 
            Rotation2d.fromRotations(angleEncoder.getPosition())
        );
    }

    public void zeroEncoders() {
        m_driveMotor.setPosition(0.0);
    }
}