// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

 TalonFX m_Intake = new TalonFX(19, "canivore");
 DigitalInput di_Intake = new DigitalInput(1);
 DigitalInput di_IntakeBottom = new DigitalInput(5);

private final StaticBrake brake = new StaticBrake();

public static boolean intakeSensor = false; 
public static boolean intakeBottomSensor = false; 

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
    public boolean bottomSensorIsBlocked() {
      return(!di_IntakeBottom.get());
    }

  @Override
  public void periodic() {
    if (AmpLift.ampLiftAboveHandOff){
      coast();
    }
    
    intakeSensor = sensorIsBlocked();
    intakeBottomSensor = bottomSensorIsBlocked();

    SmartDashboard.putNumber("Intake Actual RPM", m_Intake.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putBoolean("Intake Sensor Is Blocked",intakeSensor);
    SmartDashboard.putBoolean("Bottom Intake Sensor Is Blocked",intakeBottomSensor);
    // This method will be called once per scheduler run
  }
}
