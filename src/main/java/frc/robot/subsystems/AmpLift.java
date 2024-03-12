// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpLift extends SubsystemBase {
  /** Creates a new AmpLift. */
  
  TalonFX m_AmpLift = new TalonFX(21, "canivore");
  double liftKg = Constants.AmpLiftKg;
  private final StaticBrake AmpLiftbrake = new StaticBrake();

  public AmpLift() {
    m_AmpLift.setInverted(false);
  }


  public void setPercent(double setPercent) {
    m_AmpLift.set(setPercent);
    System.out.println("Climber Percent Set to " +  setPercent);
  }

  public void stayAtPosition(double AmpLiftKg) {
   //empty
  }

  public void brake() {
    m_AmpLift.setControl(AmpLiftbrake);
  

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
