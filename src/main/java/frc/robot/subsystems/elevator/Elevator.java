package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

  private SparkMax m_masterMotor;
  private SparkMax m_slaveMotor;

  private SparkClosedLoopController m_closedController;
  private double targetSetpoint;

  public Elevator() {
    m_masterMotor = new SparkMax(CANConstants.Elevator.kElevatorMaster, MotorType.kBrushless);
    m_slaveMotor = new SparkMax(CANConstants.Elevator.kElevatorSlave, MotorType.kBrushless);

    m_masterMotor.configure(Configs.Elevator.Master.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    m_slaveMotor.configure(Configs.Elevator.Slave.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_closedController = m_masterMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // targetSetpoint needs to be coverted from Inches to Rotations

    m_closedController.setReference((targetSetpoint / (Math.PI * ElevatorConstants.kElevatorSprocketPitchDiameter) * ElevatorConstants.kElevatorMotorReduction),
     ControlType.kPosition);
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
}
