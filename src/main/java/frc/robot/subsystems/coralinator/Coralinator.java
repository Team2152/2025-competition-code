package frc.robot.subsystems.coralinator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.CoralinatorConstants;

public class Coralinator extends SubsystemBase {
  public enum CoralinatorStates {
    EMPTY,
    INDEXING,
    READY
  }

  private SparkMax m_coralinatorMotor;

  private DigitalInput m_coralSensor;
  private CoralinatorStates m_currentState;

  public Coralinator() {
    m_coralinatorMotor = new SparkMax(CANConstants.Coralinator.kCoralinator, MotorType.kBrushless);

    m_coralSensor = new DigitalInput(0);
    m_currentState = CoralinatorStates.READY;
  }

  @Override
  public void periodic() {

  }

  public void index() {
    if (m_currentState == CoralinatorStates.READY) {
      return;
    }
  
    m_coralinatorMotor.set(CoralinatorConstants.kIndexingPower);
    m_currentState = CoralinatorStates.INDEXING;
  
    if (m_coralSensor.get()) {
      stopIndexing();
    }
  }
  
  private void stopIndexing() {
    m_coralinatorMotor.set(0);
    m_currentState = CoralinatorStates.READY;
  }
  
  public Command indexCmd() {
    return runOnce(()->{
      if (m_currentState != CoralinatorStates.READY) {
        index();
      }
    });
  }
}
