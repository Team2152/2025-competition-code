package frc.robot.subsystems.coral_manipulator;

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


public class CoralManipulator extends SubsystemBase {    
    
    private final SparkMax m_CoralManipulatorMotor;

    private final RelativeEncoder m_CoralManipulatorEncoder;

    private final PIDController m_pid;

    private double targetDistance = 0;
    
    

    public CoralManipulator(int CoralMotorID){

        m_CoralManipulatorMotor = new SparkMax(CoralMotorID, MotorType.kBrushless);
       
        m_CoralManipulatorEncoder = m_CoralManipulatorMotor.getEncoder();
        m_CoralManipulatorMotor.configure(Configs.CoralModule.CoralManipulator.config, ResetMode.kResetSafeParameters,
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
        m_CoralManipulatorMotor.set(m_pid.calculate(m_CoralManipulatorEncoder.getPosition(),targetDistance));
        System.out.println(m_pid.calculate(m_CoralManipulatorEncoder.getPosition(),targetDistance));
    }
    //m_CoralManipulatorEncoder.getPosition()
}

