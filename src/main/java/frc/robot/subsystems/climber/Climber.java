// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  public enum ClimberStates {
      IDLE,
      DEPLOYED,
      LATCHED,
      CLIMBING,
      CLIMBED
  }

  public TalonFX m_intakeMotor;
  public TalonFX m_climberMotor;

  public ClimberStates m_currentState;

  public Trigger intakingStalling;
  public boolean isIntakeStalled;

  public Climber() {
      m_intakeMotor = new TalonFX(Constants.CANConstants.Climber.kIntake);
      m_climberMotor = new TalonFX(Constants.CANConstants.Climber.kClimber);

      Trigger intakeStalling = new Trigger(this::isIntakeStalling)
        .debounce(0.75, DebounceType.kRising);
      
      intakeStalling
        .onTrue(runOnce(()->isIntakeStalled = true))
        .onFalse(runOnce(()->isIntakeStalled = false));
      
      m_currentState = ClimberStates.IDLE;
  }

  @Override
  public void periodic() {
      switch (m_currentState) {
          case DEPLOYED:
              m_intakeMotor.set(1);
              if (getPosition() >= ClimberConstants.Setpoints.kDeployed) {
                  m_climberMotor.set(0);
                  if (isIntakeStalled) {
                    m_currentState = ClimberStates.LATCHED;
                  }
              } else {
                  m_climberMotor.set(0.3);
              }
              break;

          case LATCHED:
              m_intakeMotor.set(0);
              m_currentState = ClimberStates.CLIMBING;
              break;

          case CLIMBING:
              if (getPosition() >= ClimberConstants.Setpoints.kClimbed) {
                  m_climberMotor.set(0);
                  m_currentState = ClimberStates.IDLE;
              } else {
                  m_climberMotor.set(0.3);
              }
              break;

          default:
              m_climberMotor.set(0);
              m_intakeMotor.set(0);
              break;
      }
  }

  private double getPosition() {
      return m_climberMotor.getPosition().getValueAsDouble();
  }

  private void setState(ClimberStates state) {
    if (m_currentState != ClimberStates.CLIMBED) {
      m_currentState = state;
    }  
  }

  public Command setStateCmd(ClimberStates state) {
      return runOnce(() -> setState(state));
  }

  private boolean isIntakeStalling() {
    return Math.abs(m_intakeMotor.getVelocity().getValueAsDouble()) < 10 && m_intakeMotor.get() != 0;
}

}
