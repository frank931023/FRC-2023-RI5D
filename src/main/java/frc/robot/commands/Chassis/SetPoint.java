// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ChassisSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetPoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ChassisSubsystem m_drive;

  private PIDController lockPIDLeft = new PIDController(
    DriveConstants.kLockPIDLeftkP, DriveConstants.kLockPIDLeftkI, 
    DriveConstants.kLockPIDLeftkD, DriveConstants.kLockPIDLeftiLimit);

  private PIDController lockPIDRight = new PIDController(
    DriveConstants.kLockPIDRightkP, DriveConstants.kLockPIDRightkI,
    DriveConstants.kLockPIDRightkD, DriveConstants.kLockPIDRightiLimit);

  private Joystick driverJoystick = new Joystick(OIConstants.driverController);

  private boolean isEnd = false;

  public SetPoint(ChassisSubsystem m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setMotor2zero();
    m_drive.resetEncoders();
    isEnd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(driverJoystick.getRawAxis(OIConstants.leftStick_Y)) < 0.02
    || (Math.abs(driverJoystick.getRawAxis(OIConstants.rightStick_X))) < 0.02) {
      stop();
    }

    double disLeft = m_drive.getLeftEncoder().getDistance();
    double disRight = m_drive.getRightEncoder().getDistance();

    m_drive.tankDriveVolts(lockPIDLeft.calculate(-disLeft), lockPIDRight.calculate(-disRight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isEnd;
  }

  public void stop() {
    isEnd = true;
  }
}