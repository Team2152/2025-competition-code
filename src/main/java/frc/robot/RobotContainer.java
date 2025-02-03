// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
  private static CommandXboxController m_driverController;

  private static Drivetrain m_drivetrain;

  public RobotContainer() {
    m_driverController = new CommandXboxController(0);

    m_drivetrain = new Drivetrain();
  
    configureBindings();
  }
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      m_drivetrain.teleopDrive(
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX(),
        () -> false // m_driverController.leftTrigger().getAsBoolean()
      )
    );

    m_driverController.back()
      .onTrue(m_drivetrain.zeroGyro(0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
