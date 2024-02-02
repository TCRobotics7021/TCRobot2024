// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
 // TalonFX m_Intake = new TalonFX(55);
  public Intake() {}
  /* 
  public void setSpeed(double intakeSpeed) {
    m_Intake.set(intakeSpeed);
  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
