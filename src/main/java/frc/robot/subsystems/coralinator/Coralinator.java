package frc.robot.subsystems.coralinator;

import com.revrobotics.spark.SparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.CoralinatorConstants;

public class Coralinator extends SubsystemBase {
  public enum CoralinatorStates {
    EMPTY,
    INDEXING,
    PULLBACK,
    READY
  }

  private SparkMax m_coralinatorMotor;

  private TimeOfFlight m_coralSensor;
  public CoralinatorStates m_currentState;
  public boolean hasCoral;
  private boolean cooking = true;

  public Coralinator() {
    m_coralinatorMotor = new SparkMax(CANConstants.Coralinator.kCoralinator, MotorType.kBrushless);

    m_coralSensor = new TimeOfFlight(14);
    m_coralSensor.setRangingMode(RangingMode.Short, 24);
    m_currentState = CoralinatorStates.READY;
    hasCoral = m_currentState == CoralinatorStates.READY;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coralator TOF", m_coralSensor.getRange());
    if (cooking) {
      if (m_currentState == CoralinatorStates.EMPTY) {
        if (checkBeambreak()) {
          m_currentState = CoralinatorStates.INDEXING;
        }
      }
      if (m_currentState == CoralinatorStates.INDEXING) {
        index();
      }
      if (m_currentState == CoralinatorStates.PULLBACK) {
        pullback();
      }
    }

    if (m_currentState == CoralinatorStates.READY) {
      hasCoral = true;
    }
  }

  public void index() {
    if (checkBeambreak()) {
      m_coralinatorMotor.set(CoralinatorConstants.kIndexingPower);
    } else {
      m_currentState = CoralinatorStates.PULLBACK;
      m_coralinatorMotor.set(0);
    }
  }

  public void pullback() {
    if (!checkBeambreak()) {
      m_coralinatorMotor.set(-(CoralinatorConstants.kIndexingPower/2));
    } else {
      m_currentState = CoralinatorStates.READY;
      m_coralinatorMotor.set(0);
    }
  }

  public boolean checkBeambreak() {
    return m_coralSensor.getRange() < 200;
  }

  public Command run(double speed) {
    return runOnce(()->m_coralinatorMotor.set(speed));
  }

  public Command score(double speed) {
    return runOnce(()->{
      m_currentState = CoralinatorStates.EMPTY;
    });
  }

  public boolean hasCoral() {
    return (m_currentState == CoralinatorStates.READY);
  }
}
