// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
 TalonFX m_ShooterTop = new TalonFX(17,"canivore");
 TalonFX m_ShooterBottom = new TalonFX(18,"canivore");
  TalonFX m_ShooterPitch = new TalonFX(15, "canivore");
  CANcoder CancoderPitch = new CANcoder(16, "canivore");
   private final VelocityVoltage VoltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
   private final NeutralOut brake = new NeutralOut();
   private final PositionVoltage VoltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);


  public Shooter(){
    SmartDashboard.putNumber("Set Top RPM", 3000);
    SmartDashboard.putNumber("Set Bottom RPM", 3000);
   
        TalonFXConfiguration configsShooter = new TalonFXConfiguration();



    /* Voltage-based velocity requires  feed forward to account for the back-emf of the motor */
    configsShooter.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configsShooter.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configsShooter.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configsShooter.Slot0.kV = 0.12; // Falcon 500 is  500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configsShooter.Voltage.PeakForwardVoltage = 100;
    configsShooter.Voltage.PeakReverseVoltage = -100;


    
  
    TalonFXConfiguration configsPitch = new TalonFXConfiguration();
    configsPitch.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configsPitch.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configsPitch.Voltage.PeakForwardVoltage = 8;
    configsPitch.Voltage.PeakReverseVoltage = -8;
    
    configsPitch.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    configsPitch.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    configsPitch.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    configsPitch.TorqueCurrent.PeakReverseTorqueCurrent = 130;
  
    configsPitch.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configsPitch.Feedback.FeedbackRemoteSensorID = CancoderPitch.getDeviceID();
  
    
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
     StatusCode status2 = StatusCode.StatusCodeNotInitialized;
     StatusCode status3 = StatusCode.StatusCodeNotInitialized;
    
     for (int i = 0; i < 5; ++i) {
      status1 = m_ShooterTop.getConfigurator().apply(configsShooter);
      status2 = m_ShooterBottom.getConfigurator().apply(configsShooter);
      status3 = m_ShooterPitch.getConfigurator().apply(configsPitch);
      if (status1.isOK() && status2.isOK() && status3.isOK()) break;
    }

    if(!status1.isOK()) {
      System.out.println("Could not apply configs to m_ShooterTop, error code: " + status1.toString());
    } 
    if(!status2.isOK()) {
      System.out.println("Could not apply configs to m_ShooterBottom, error code: " + status2.toString());
    }     
    if(!status3.isOK()) {
      System.out.println("Could not apply configs to m_ShooterPitch, error code: " + status3.toString());
    }   

    
    m_ShooterBottom.setInverted(true);
    
  }

  public double getPitch(){
    return (Rotation2d.fromRotations(CancoderPitch.getAbsolutePosition().getValueAsDouble()).getDegrees() + Constants.shooterPitchCancoderCal);
  }
  
  public void setPitch(double targetPitch){
     m_ShooterPitch.setControl(VoltagePosition.withPosition(targetPitch - Constants.shooterPitchCancoderCal));
   }

 
  public void coastPitch(){
   // m_ShooterPitch.setControl(brake);
    
  }

  public boolean atSpeed(double bottomTargetSpeed, double topTargetSpeed) {
   double topCurrentSpeed = m_ShooterTop.getVelocity().getValueAsDouble()*60;
   double bottomCurrentSpeed = m_ShooterBottom.getVelocity().getValueAsDouble()*60;
   
    return(topCurrentSpeed > topTargetSpeed - 100 && bottomCurrentSpeed > bottomTargetSpeed - 100);
  }


  public void setRPM(double Top_RPM, double Bottom_RPM) {
    m_ShooterTop.setControl(VoltageVelocity.withVelocity(Top_RPM));
    m_ShooterBottom.setControl(VoltageVelocity.withVelocity(Bottom_RPM));

  }
  public void coast() {
    m_ShooterTop.setControl(brake);
    m_ShooterBottom.setControl(brake);
  }

  public void setPercent(double shooterPercent) {
    m_ShooterTop.set(shooterPercent);
    m_ShooterBottom.set(-shooterPercent);
  }

  public void shooterAngleFromDistance() {
    //insert quadratic formula here

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Bottom Actual RPM", m_ShooterBottom.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Shooter Top Actual RPM", m_ShooterTop.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Shooter Pitch Angle", getPitch());
  }
}
