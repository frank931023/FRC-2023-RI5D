// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.motorID, MotorType.kBrushless);
  
  /** Creates a new LufySubsystem. */
  public ArmSubsystem() {
    m_armMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed){
    m_armMotor.set(speed);
  }
  
  public void stop(){
    m_armMotor.set(0);
  }
}
