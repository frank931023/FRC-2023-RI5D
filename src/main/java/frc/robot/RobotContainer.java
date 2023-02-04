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
    // m_luffy.setDefaultCommand(new RunCommand(() -> {
    //   m_luffy.run(operatorJoystick.getRawAxis(OIConstants.leftStick_Y) * LuffyConstants.strechSpeedScaler); }
    //   , m_luffy));

    // Elevator
    // m_elevator.setDefaultCommand(new RunCommand(() -> {
    //   m_elevator.run(operatorJoystick.getRawAxis(OIConstants.rightStick_Y) * ElevatorConstants.elevatorSpeedScaler); }
    //   , m_elevator));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  // Use this method to define your button->command mappings. 
  private void configureButtonBindings() {
    // new JoystickButton(driverJoystick, OIConstants.Btn_A).toggleOnTrue(m_setPoint);
    // new JoystickButton(operatorJoystick, OIConstants.Btn_A).onTrue(new GrabAndRelease(m_grab)); 
    // new JoystickButton(operatorJoystick, OIConstants.Btn_B).onTrue(new UpAndDown(m_ototakeHirotada)); 
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

// Driver Station reported IP: 10.71.30.2

// > Task :discoverroborio
// Discovering Target roborio
// admin @ null: Connected.
//   Reason: InvalidImageException
//   Invalid RoboRIO Image Version!
// RoboRIO image and GradleRIO versions are incompatible:
//         Current image version: 2022_v4.0
//         GradleRIO-compatible image versions: 2023_v3.*
// See https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.htmlfor information about upgrading the RoboRIO image.
// See https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html and
// https://docs.wpilib.org/en/stable/docs/software/vscode-overview/importing-gradle-project.html
// for information about updating WPILib and GradleRIO.
// admin @ roborio-7130-FRC.local: Connected.
//   Reason: InvalidImageException
//   Invalid RoboRIO Image Version!
// RoboRIO image and GradleRIO versions are incompatible:
//         Current image version: 2022_v4.0
//         GradleRIO-compatible image versions: 2023_v3.*
// See https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.htmlfor information about upgrading the RoboRIO image.
// See https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html and
// https://docs.wpilib.org/en/stable/docs/software/vscode-overview/importing-gradle-project.html
// for information about updating WPILib and GradleRIO.
// admin @ 10.71.30.2: Connected.
//   Reason: InvalidImageException
//   Invalid RoboRIO Image Version!
// RoboRIO image and GradleRIO versions are incompatible:
//         Current image version: 2022_v4.0
//         GradleRIO-compatible image versions: 2023_v3.*
// See https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.htmlfor information about upgrading the RoboRIO image.
// See https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html and
// https://docs.wpilib.org/en/stable/docs/software/vscode-overview/importing-gradle-project.html
// for information about updating WPILib and GradleRIO.
// 1 other action(s) resolved but not connected.
// 3 other action(s) failed resolution.
// Run with --info for more details


// > Task :discoverroborio FAILED
// Missing Target!
// =============================================
// Are you connected to the robot, and is it on?
// =============================================
// GradleRIO detected this build failed due to not being able to find "roborio"!
// Scroll up in this error log for more information.

// FAILURE: Build failed with an exception.

// * What went wrong:
// Execution failed for task ':discoverroborio'.
// > A failure occurred while executing edu.wpi.first.deployutils.deploy.target.discovery.TargetDiscoveryWorker
//    > Target roborio could not be found at any location! See above for more details.

// * Try:
// > Run with --stacktrace option to get the stack trace.
// > Run with --info or --debug option to get more log output.
// > Run with --scan to get full insights.

// * Get more help at https://help.gradle.org

// BUILD FAILED in 3s
// 4 actionable tasks: 2 executed, 2 up-to-date

//  *  The terminal process "cmd.exe /d /c gradlew deploy  -PteamNumber=7130 --offline  -Dorg.gradle.java.home="C:\Users\Public\wpilib\2023\jdk"" terminated with exit code: 1. 
//  *  Terminal will be reused by tasks, press any key to close it. 










