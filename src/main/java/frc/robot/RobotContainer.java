// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberStates;
import frc.robot.subsystems.coralinator.Coralinator;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.AlignmentStatus;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final SendableChooser<Command> m_autoChooser;

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  private final Coralinator m_coralinator = new Coralinator();
  private final LEDs m_leds = new LEDs(4, m_coralinator);
  private final Climber m_climber = new Climber();
  
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(1);

  public RobotContainer() {
    NamedCommands.registerCommand("EL-L2", m_elevator.setHeightCmd(ElevatorHeight.L2));
    NamedCommands.registerCommand("EL-L3", m_elevator.setHeightCmd(ElevatorHeight.L3));
    NamedCommands.registerCommand("EL-INTAKE", m_elevator.setHeightCmd(ElevatorHeight.INTAKE));

    NamedCommands.registerCommand("SCORE", 
      m_coralinator.run(1)
      .andThen(new WaitCommand(1)
      .andThen(m_coralinator.score(0)
      .andThen(m_coralinator.run(0))))
    );

    NamedCommands.registerCommand("ALIGN-LEFT", m_drivetrain.setAlignmentState(AlignmentStatus.LEFT));
    NamedCommands.registerCommand("ALIGN-RIGHT", m_drivetrain.setAlignmentState(AlignmentStatus.RIGHT));
    NamedCommands.registerCommand("ALIGN-NONE", m_drivetrain.setAlignmentState(AlignmentStatus.NONE));
    NamedCommands.registerCommand("INTAKE-WAIT", new WaitUntilCommand(() -> m_coralinator.hasCoral));

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

    m_driverController.y()
      .onTrue(m_climber.setStateCmd(ClimberStates.DEPLOYED));
  
    m_operatorController.povLeft()
      .onTrue(m_elevator.setHeightCmd(ElevatorHeight.INTAKE));

    m_operatorController.povUp()
      .onTrue(m_elevator.setHeightCmd(ElevatorHeight.ZERO
      ));

    m_operatorController.povRight()
      .onTrue(m_elevator.setHeightCmd(ElevatorHeight.L3));

    m_operatorController.povDown()
      .onTrue(m_elevator.setHeightCmd(ElevatorHeight.L2));

    m_operatorController.rightTrigger()
      .onTrue(m_elevator.modifyTarget(.05));

    m_operatorController.leftTrigger()
      .onTrue(m_elevator.modifyTarget(-.05));

    m_operatorController.leftBumper()
      .whileTrue(m_coralinator.run(-1))
      .whileFalse(m_coralinator.run(0));

    m_operatorController.a()
      .onTrue(m_coralinator.run(1)
      .andThen(new WaitCommand(1.0)
      .andThen(m_coralinator.score(0))
      .andThen(m_coralinator.run(0))));

   
   //   m_operatorController.back()
     // .onTrue(m_elevator.)

    m_driverController.a()
      .whileTrue(m_drivetrain.setX());

    m_driverController.leftBumper()
      .onTrue(m_drivetrain.setAlignmentState(AlignmentStatus.LEFT))
      .onFalse(m_drivetrain.setAlignmentState(AlignmentStatus.NONE));

    m_driverController.rightBumper()
      .onTrue(m_drivetrain.setAlignmentState(AlignmentStatus.RIGHT))
      .onFalse(m_drivetrain.setAlignmentState(AlignmentStatus.NONE));
  }
  

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}