// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Chassis.PathFollowingRamsete;
import frc.robot.commands.Chassis.LockPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Joystick
  private final Joystick driverJoystick = new Joystick(OIConstants.driverController);
  private final Joystick operatorJoystick = new Joystick(OIConstants.operatorController);

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // Chassis Commands
  private final LockPID m_setPoint = new LockPID(m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drive
    m_drive.setDefaultCommand(new RunCommand(() -> {
      m_drive.arcadeDrive(
        driverJoystick.getRawAxis(OIConstants.leftStick_Y) * DriveConstants.chassisArcadeSpdScaler, 
        driverJoystick.getRawAxis(OIConstants.rightStick_X) * DriveConstants.chassisArcadeRotScaler);
    }, m_drive));

    // Elevator
    m_elevator.setDefaultCommand(new RunCommand(() -> {
      m_elevator.run(operatorJoystick.getRawAxis(OIConstants.rightStick_Y) * ElevatorConstants.elevatorSpeedScaler); }
      , m_elevator));


    // Configure the button bindings
    configureButtonBindings();

    PathPlannerServer.startServer(7130);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, OIConstants.Btn_A).onTrue(m_setPoint);
    new JoystickButton(driverJoystick, OIConstants.Btn_B).onTrue(new RunCommand( () -> {m_drive.resetEncoders();}, m_drive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new PathFollowingRamsete(m_drive, "New Path", true), 
      m_setPoint);
  }
}