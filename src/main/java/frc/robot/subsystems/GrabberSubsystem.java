// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  private final Compressor comp = new Compressor(GrabberConstants.compressorID ,PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid DoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.ForwardChannel, GrabberConstants.ReverseChannel);
  private final CANSparkMax m_grabberMotor = new CANSparkMax(GrabberConstants.motorID, MotorType.kBrushless);

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    m_grabberMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enablecompressor(){
    comp.enableDigital();
    DoublePCM.isFwdSolenoidDisabled();
  }

  public void handOpen(){
    DoublePCM.set(DoubleSolenoid.Value.kForward);
  }

  public void handClose(){
    DoublePCM.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void handStop(){
    DoublePCM.set(DoubleSolenoid.Value.kOff);
  }

  public void rollIn(){
    m_grabberMotor.set(GrabberConstants.motorSpeed);
  }

  public void rollOut(){
    m_grabberMotor.set(-GrabberConstants.motorSpeed);
  }

  public void rollStop(){
    m_grabberMotor.set(0);
  }
}
