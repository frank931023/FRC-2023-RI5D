// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  CANSparkMax m_motorElevatorLeft = new CANSparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
  CANSparkMax m_motorElevatorRight = new CANSparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motorElevatorLeft.setInverted(false);
    m_motorElevatorRight.follow(m_motorElevatorLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed){
    m_motorElevatorLeft.set(speed);
  }
  
  public void stop(){
    m_motorElevatorLeft.set(0);
  }
}
