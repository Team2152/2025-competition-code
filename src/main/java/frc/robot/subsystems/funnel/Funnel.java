// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANConstants;

public class Funnel extends SubsystemBase {
  /** Creates a new Funnel. */
  private SparkMax m_funnelMotor;
  private SparkClosedLoopController m_funnelController;
  private double kStow = 1;
  private double kDeployed = -20;

  private boolean deployed = false;
  public Funnel() {
    m_funnelMotor = new SparkMax(CANConstants.Funnel.kFunnel, MotorType.kBrushless);
    m_funnelMotor.configure(Configs.Funnel.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_funnelController = m_funnelMotor.getClosedLoopController();
    //m_funnelPID = new PIDController(.15, 0, 0);
    m_funnelController.setReference(kStow, ControlType.kPosition);
  }

  @Override
  public void periodic() {
  }

  public Command toggleFunnel() {
    return runOnce(()->{
      deployed = !deployed;

      if (deployed) {
        m_funnelController.setReference(kDeployed, ControlType.kPosition);
      } else {
        m_funnelController.setReference(kStow, ControlType.kPosition);
      }
    });
  }
}
