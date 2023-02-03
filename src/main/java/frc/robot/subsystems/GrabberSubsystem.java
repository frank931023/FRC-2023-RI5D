// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  private final Compressor comp = new Compressor(GrabberConstants.compressorID ,PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid DoublePCM1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.DoublePCM1ForwardChannel, GrabberConstants.DoublePCM1ReverseChannel);
  private final DoubleSolenoid DoublePCM2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.DoublePCM2ForwardChannel, GrabberConstants.DoublePCM2ReverseChannel);

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enablecompressor(){
    comp.enableDigital();
    DoublePCM1.isFwdSolenoidDisabled();
    DoublePCM2.isFwdSolenoidDisabled();
  }

  public void forward(){
    DoublePCM1.set(DoubleSolenoid.Value.kForward);
    DoublePCM2.set(DoubleSolenoid.Value.kForward);
  }

  public void reverse(){
    DoublePCM1.set(DoubleSolenoid.Value.kReverse);
    DoublePCM2.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void stop(){
    DoublePCM1.set(DoubleSolenoid.Value.kOff);
    DoublePCM2.set(DoubleSolenoid.Value.kOff);
  }
}
