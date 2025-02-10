package frc.robot.subsystems.funnel;

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


public class Funnel extends SubsystemBase {    
    
    private final SparkMax m_FunnelMotor;

    private final RelativeEncoder m_FunnelEncoder;

    private final PIDController m_pid;

    private double targetDistance = 0;
    
    

    public Funnel(int BeltMotorCanID){

        m_FunnelMotor = new SparkMax(BeltMotorCanID, MotorType.kBrushless);
       
        m_FunnelEncoder = m_FunnelMotor.getEncoder();
        m_FunnelMotor.configure(Configs.FunnelModule.Funnel.config, ResetMode.kResetSafeParameters,
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
        m_FunnelMotor.set(m_pid.calculate(m_FunnelEncoder.getPosition(),targetDistance));
        System.out.println(m_pid.calculate(m_FunnelEncoder.getPosition(),targetDistance));
    }
    //m_FunnelEncoder.getPosition()
}

