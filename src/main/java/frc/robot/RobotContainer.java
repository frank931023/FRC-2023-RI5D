// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LuffyConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Arm.GrabAndRelease;
import frc.robot.commands.Arm.UpAndDown;
import frc.robot.commands.Chassis.PathFollowing;
import frc.robot.commands.Chassis.SetPoint;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LuffySubsystem;
import frc.robot.subsystems.OtotakeHirotadaSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final ChassisSubsystem m_drive = new ChassisSubsystem();
  private final GrabberSubsystem m_grab = new GrabberSubsystem();
  private final LuffySubsystem m_luffy = new LuffySubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final OtotakeHirotadaSubsystem m_ototakeHirotada = new OtotakeHirotadaSubsystem();

  // Chassis Commands
  private final SetPoint m_setPoint = new SetPoint(m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drive
    m_drive.setDefaultCommand(new RunCommand(() -> {
      m_drive.arcadeDrive(
        -driverJoystick.getRawAxis(OIConstants.leftStick_Y) * DriveConstants.chassisSpeedScaler, driverJoystick.getRawAxis(OIConstants.rightStick_X) * DriveConstants.chassisSpeedScaler);}
      , m_drive));
    
    // Strech 
    m_luffy.setDefaultCommand(new RunCommand(() -> {
      m_luffy.run(operatorJoystick.getRawAxis(OIConstants.leftStick_Y) * LuffyConstants.strechSpeedScaler); }
      , m_luffy));

    // Elevator
    m_elevator.setDefaultCommand(new RunCommand(() -> {
      m_elevator.run(operatorJoystick.getRawAxis(OIConstants.rightStick_Y) * ElevatorConstants.elevatorSpeedScaler); }
      , m_elevator));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  // Use this method to define your button->command mappings. 
  private void configureButtonBindings() {
    // new JoystickButton(driverJoystick, OIConstants.Btn_A).toggleOnTrue(m_setPoint);
    new JoystickButton(operatorJoystick, OIConstants.Btn_A).onTrue(new GrabAndRelease(m_grab)); 
    new JoystickButton(operatorJoystick, OIConstants.Btn_B).onTrue(new UpAndDown(m_ototakeHirotada)); 
  }

  // Use this to pass the autonomous command to the main {@link Robot} class.
  public Command getAutonomousCommand() {
    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // PathPlannerTrajectory path1 = 
    //   PathPlanner.loadPath(
    //     "New Path", new PathConstraints(
    //       AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    // RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //         path1,
    //         m_drive::getPose,
    //         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         m_drive::getWheelSpeeds,
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         m_drive::tankDriveVolts,
    //         m_drive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_drive.resetOdometry(path1.getInitialPose());

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));

    return new SequentialCommandGroup(
      new PathFollowing(m_drive, "New Path"), m_setPoint);
  }
}