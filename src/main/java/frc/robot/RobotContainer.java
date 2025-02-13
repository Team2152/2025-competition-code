// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final SendableChooser<Command> m_autoChooser;

  private final Drivetrain m_robotDrive = new Drivetrain();
  // private final Elevator m_elevator = new Elevator(CANConstants.Elevator.kLeftElevatorMotorId, CANConstants.Elevator.kRightElevatorMotorId);
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Routine", m_autoChooser);

    configureBindings();

    m_robotDrive.setDefaultCommand(
        m_robotDrive.teleopDrive(
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX(),
                () -> m_driverController.leftTrigger().getAsBoolean()
            ));
      // m_driverController.a() 
      //   .onTrue(m_elevator.setPosition(100));
      //   m_driverController.b() 
      //   .onTrue(m_elevator.setPosition(50));
  }

  private void configureBindings() {
    m_driverController.back()
      .onTrue(m_robotDrive.zeroHeading());
  
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}