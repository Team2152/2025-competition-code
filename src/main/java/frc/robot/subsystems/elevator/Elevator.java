package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.util.SimpleBeanPropertyDefinition;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  public enum ElevatorHeight {
    ZERO,
    INTAKE,
    ALIGN,
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

  private Mechanism2d mech;
  private MechanismRoot2d root;
  private MechanismLigament2d elevatorStage;
  
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
      mech = new Mechanism2d(3, 3);
      root = mech.getRoot("elevator", 1.75, 0);

      elevatorStage =
          root.append(new MechanismLigament2d("elevator1", 1, 90, 8, new Color8Bit(Color.kBlue)));
      SmartDashboard.putData("Elevator Simulation", mech);
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

    elevatorStage.setLength(targetSetpoint * Math.PI * Units.inchesToMeters(1.25));

    // if (m_limitSwitch.get()) {
    //   lsCycles++;
    // }

    // if (lsCycles>10) {
    //   m_masterMotor.setPosition(0);
    //   lsCycles=0;
    // }


    
  }

  public void setHeight(ElevatorHeight setpoint) {
    switch (setpoint) {
      case ZERO:
        targetSetpoint = -0.1;
        break;
      case INTAKE:
        targetSetpoint = ElevatorConstants.Setpoints.kIntake;
        break;
      case ALIGN:
        targetSetpoint = ElevatorConstants.Setpoints.kAlign;
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

  public Command zeroTarget() {
    return runOnce(()->targetSetpoint=0);
  }
}
