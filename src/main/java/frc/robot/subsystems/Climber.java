//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Lift. */
  TalonFX m_Climber = new TalonFX(21, "canivore");
  double liftKg = Constants.ClimberKg;
  private final StaticBrake brake = new StaticBrake();

  public Climber() {
    m_Climber.setInverted(false);
  }


  public void setPercent(double setPercent) {
    m_Climber.set(setPercent);
    System.out.println("Climber Percent Set to " +  setPercent);
  }

  public void stayAtPosition(double ClimberKg) {
    //m_Climber.set(ClimberKg);
  }

  public void brake() {
    m_Climber.setControl(brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
