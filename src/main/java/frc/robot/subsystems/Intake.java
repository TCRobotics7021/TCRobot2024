// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
 TalonFX m_Intake = new TalonFX(19, "canivore");
 DigitalInput di_Intake = new DigitalInput(1);
 private final VelocityVoltage VoltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
   private final NeutralOut brake = new NeutralOut();
   private final PositionVoltage VoltageIntake = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
   public static boolean intakeSensor = false; 
  /** Creates a new Intake. */

  public Intake() {
    SmartDashboard.putNumber("Set Intake RPM", 0);
    m_Intake.setInverted(false);
  }
  /* 
  public void setSpeed(double intakeSpeed) {
    m_Intake.set(intakeSpeed);
  }
*/

  public void coast() {
      m_Intake.setControl(brake);
    }

    public void setPercent(double shooterPercent) {
      if (!AmpLift.ampLiftAboveHandOff){
          m_Intake.set(shooterPercent);
      }
    
    }

    public boolean sensorIsBlocked() {
      return(!di_Intake.get());
    }


  @Override
  public void periodic() {
    if (AmpLift.ampLiftAboveHandOff){
      coast();
    }
    
    intakeSensor = sensorIsBlocked();

    SmartDashboard.putNumber("Intake Actual RPM", m_Intake.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putBoolean("Intake Sensor Is Blocked",intakeSensor);
    // This method will be called once per scheduler run
  }
}
