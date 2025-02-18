// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final SendableChooser<Command> m_autoChooser;

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  // private final Elevator m_elevator = new Elevator(CANConstants.Elevator.kLeftElevatorMotorId, CANConstants.Elevator.kRightElevatorMotorId);
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Routine", m_autoChooser);

    configureBindings();

    m_drivetrain.setDefaultCommand(
      m_drivetrain.teleopDrive(
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                () -> m_driverController.leftTrigger().getAsBoolean()
            ));
  }

  private void configureBindings() {
    m_driverController.back()
      .onTrue(m_drivetrain.zeroHeading());
  
    m_driverController.povLeft()
      .onTrue(m_elevator.setHeightCmd(ElevatorHeight.INTAKE));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}