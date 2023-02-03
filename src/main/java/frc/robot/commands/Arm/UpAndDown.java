// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OtotakeHirotadaSubsystem;

public class UpAndDown extends CommandBase {

  private final OtotakeHirotadaSubsystem m_OtotakeHirotadaSubsystem;
  private boolean state;

  /** Creates a new UpAndDown. */
  public UpAndDown(OtotakeHirotadaSubsystem m_OtotakeHirotadaSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_OtotakeHirotadaSubsystem = m_OtotakeHirotadaSubsystem;
    addRequirements(m_OtotakeHirotadaSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == false){
      m_OtotakeHirotadaSubsystem.reverse();
    } else if (state == true){
      m_OtotakeHirotadaSubsystem.forward();
    }
    state = !state;
    System.out.println(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
