// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Grabber;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.PIDConstants;
// import frc.robot.subsystems.GrabberWheelSubsystem;

// public class GrabberWheel extends CommandBase {

//   private final GrabberWheelSubsystem grabWheel;
//   private final PIDController pid = new PIDController(PIDConstants.kD_grabberWheel, PIDConstants.kI_grabberWheel, PIDConstants.kD_grabberWheel);

//   /** Creates a new GrabberWheel. */
//   public GrabberWheel(GrabberWheelSubsystem grabWheel) {
//     this.grabWheel = grabWheel;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(grabWheel);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     grabWheel.rollRun(pid.calculate(grabWheel.getWheelVel(), PIDConstants.grabberWheelSetPoint));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }