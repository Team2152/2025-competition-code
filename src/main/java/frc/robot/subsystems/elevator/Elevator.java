package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  public enum ElevatorHeight {
    INTAKE,
    L1,
    L2,
    L3,
    L4
  }

  private TalonFX m_masterMotor;
  private TalonFX m_slaveMotor;
  private Follower m_follower;
  private MotionMagicVoltage m_request;

  private DigitalInput m_limitSwitch;

  private double previousSetpoint;
  private double targetSetpoint;
  private double lsCycles;
  
  public Elevator() {
    m_masterMotor = new TalonFX(CANConstants.Elevator.kElevatorMaster);
    m_slaveMotor = new TalonFX(CANConstants.Elevator.kElevatorSlave);
    m_follower = new Follower(CANConstants.Elevator.kElevatorMaster, false);
    m_slaveMotor.setControl(m_follower);

    m_masterMotor.getConfigurator().apply(Configs.Elevator.Master.config);
    
    m_slaveMotor.getConfigurator().apply(Configs.Elevator.Slave.config);
    m_request = new MotionMagicVoltage(0);
    m_masterMotor.setPosition(0);

    m_limitSwitch = new DigitalInput(1);
    lsCycles = 0;

    //setHeight(ElevatorHeight.INTAKE);
  }

  @Override
  public void periodic() {
    if (targetSetpoint != previousSetpoint) {
        m_masterMotor.setControl(m_request.withPosition(targetSetpoint));
        previousSetpoint = targetSetpoint;
    }
    SmartDashboard.putNumber("Elevator Pos", m_masterMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Target", targetSetpoint);
    SmartDashboard.putBoolean("Elevator LS", m_limitSwitch.get());

    if (m_limitSwitch.get()) {
      lsCycles++;
    }

    if (lsCycles>100) {
      m_masterMotor.setPosition(0);
      lsCycles=0;
    }


    
  }

  public void setHeight(ElevatorHeight setpoint) {
    switch (setpoint) {
      case INTAKE:
        targetSetpoint = ElevatorConstants.Setpoints.kIntake;
        break;
      case L1:
        targetSetpoint = ElevatorConstants.Setpoints.kL1;
        break;
      case L2:
        targetSetpoint = ElevatorConstants.Setpoints.kL2;
        break;
      case L3:
        targetSetpoint = ElevatorConstants.Setpoints.kL3;
        break;
      case L4:
        targetSetpoint = ElevatorConstants.Setpoints.kL4;
        break;
    }
  }

  public Command setHeightCmd(ElevatorHeight setpoint) {
    return runOnce(()->setHeight(setpoint));
  }

  public Command modifyTarget(double amount) {
    return runOnce(()->targetSetpoint=targetSetpoint+amount);
  }
}
