package frc.robot.subsystems.elevator;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.RelativeEncoder;


public class Elevator extends SubsystemBase {    
    
    private final SparkMax m_masterEMotor;
    private final SparkMax m_followerEMotor;
    private final RelativeEncoder m_elevatorEncoder;

    private final PIDController m_pid;

    private double targetDistance = 0;

    
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.22, 0.05, 0.05);
    


    public Elevator(int LeftElevatorMotorCanID, int RigthElevatorMotorCanID){

        m_masterEMotor = new SparkMax(LeftElevatorMotorCanID, MotorType.kBrushless);
        m_followerEMotor = new SparkMax(RigthElevatorMotorCanID, MotorType.kBrushless);
        m_elevatorEncoder = m_masterEMotor.getEncoder();
        //m_elevatorEncoder = m_masterEMotor.getEncoder();
        m_masterEMotor.configure(Configs.ElevatorModule.Elevator.configM, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_followerEMotor.configure(Configs.ElevatorModule.Elevator.configF, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_pid = new PIDController(1, 0, 0);

    }

    

    public Command setPosition(double distance) {
        return runOnce(()-> {
            targetDistance = distance;
            
        });
    }

    @Override

    public void periodic() {
        m_masterEMotor.set(m_pid.calculate(m_elevatorEncoder.getPosition()));
        //System.out.println(m_pid.calculate());
    }
    
}

