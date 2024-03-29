// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AmpLift extends SubsystemBase {
  /** Creates a new AmpLift. */

  //Amp Shoot Things
  TalonFX m_AmpRoller = new TalonFX(23, "canivore");
  private final NeutralOut coast = new NeutralOut();
  

  
  TalonFX m_AmpLift = new TalonFX(22, "canivore");
  double liftKg = Constants.AmpLiftKg;
  private final StaticBrake AmpLiftbrake = new StaticBrake();
  private final PositionVoltage VoltagePosition = new PositionVoltage(0, 10, false, 0, 0, false, false, false);
  public static boolean ampLiftAboveHandOff = false;
 // DigitalInput upperLimit = new DigitalInput(5);
  DigitalInput lowerLimit = new DigitalInput(4);


  public AmpLift() {
    SmartDashboard.putNumber("AmpLiftconfigs_P", Constants.AmpLiftconfigs_P); // An error of 0.5 rotations results in 1.2 volts output
    SmartDashboard.putNumber("AmpLiftconfigs_I", Constants.AmpLiftconfigs_I);
    SmartDashboard.putNumber("AmpLiftconfigs_D", Constants.AmpLiftconfigs_D); // A change of 1 rotation per second results in 0.1 volts output
    SmartDashboard.putNumber("AmpLiftconfigs_kG", Constants.AmpLiftconfigs_kG);
    SmartDashboard.putNumber("AmpLiftconfigs_kS", Constants.AmpLiftconfigs_kS);
    m_AmpLift.setInverted(false);
    TalonFXConfiguration configsAmpLift = new TalonFXConfiguration();
    configsAmpLift.Slot0.kP = Constants.AmpLiftconfigs_P; // An error of 1 rotation per second results in 2V output
    configsAmpLift.Slot0.kI = Constants.AmpLiftconfigs_I; // An error of 1 rotation per second increases output by 0.5V every second
    configsAmpLift.Slot0.kD = Constants.AmpLiftconfigs_D; // A change of 1 rotation per second squared results in 0.01 volts output
    configsAmpLift.Slot0.kG = Constants.AmpLiftconfigs_kG;
    configsAmpLift.Slot0.kS = Constants.AmpLiftconfigs_kS;
    configsAmpLift.Voltage.PeakForwardVoltage = Constants.ampLiftPeakVvoltage;
    configsAmpLift.Voltage.PeakReverseVoltage = -Constants.ampLiftPeakVvoltage;
    
    
    
    applyConfigs(m_AmpLift, configsAmpLift, " m_AmpLift");

    m_AmpLift.setInverted(true);
 
    m_AmpRoller.setInverted(true);
  }


  public void setPercentRollers(double setPercent) {
    if(setPercent == 0){
      m_AmpRoller.setControl(AmpLiftbrake);
    } else {
      m_AmpRoller.set(setPercent);
    } 
  }

  public void reapplyConfigs() {
    TalonFXConfiguration configsAmpLift = new TalonFXConfiguration();
    configsAmpLift.Slot0.kP = SmartDashboard.getNumber("AmpLiftconfigs_P", Constants.AmpLiftconfigs_P); // An error of 0.5 rotations results in 1.2 volts output
    configsAmpLift.Slot0.kI = SmartDashboard.getNumber("AmpLiftconfigs_I", Constants.AmpLiftconfigs_I);
    configsAmpLift.Slot0.kD = SmartDashboard.getNumber("AmpLiftconfigs_D", Constants.AmpLiftconfigs_D); // A change of 1 rotation per second results in 0.1 volts output
    configsAmpLift.Slot0.kG = SmartDashboard.getNumber("AmpLiftconfigs_kG", Constants.AmpLiftconfigs_kG);
    configsAmpLift.Slot0.kS = SmartDashboard.getNumber("AmpLiftconfigs_kS", Constants.AmpLiftconfigs_kS);
    configsAmpLift.Voltage.PeakForwardVoltage = Constants.ampLiftPeakVvoltage;
    configsAmpLift.Voltage.PeakReverseVoltage = -Constants.ampLiftPeakVvoltage;
    applyConfigs(m_AmpLift, configsAmpLift, "m_AmpLiftconfigs");
        m_AmpLift.setInverted(true);

  }
  public void applyConfigs(TalonFX motor, TalonFXConfiguration configs, String motorName) {
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = motor.getConfigurator().apply(configs);
      if (status1.isOK()) break;
    }
      if(!status1.isOK()) {
      System.out.println("Could not apply Configs to " + motorName + ":" + status1.toString());
    } else {
      System.out.println("Configs successfully applied to " + motorName);
    }

  } 

  public void setPercentLift(double setPercent) {
    if (setPercent < 0 && (atLowerLimit()||getPosition()<Constants.AmpLiftMINPos)) {
      setBrakeLift();
    } else if (setPercent > 0 && (atUpperLimit()||getPosition()>Constants.AmpLiftMAXPos)){
      setBrakeLift();
    } else {
      m_AmpLift.set(setPercent);
    }
  }
  
  public void setPosition(double setPosition){
    
    if (Constants.AmpLiftMAXPos >= setPosition && setPosition >= Constants.AmpLiftMINPos) {
      m_AmpLift.setControl(VoltagePosition.withPosition(setPosition * Constants.AmpLiftRotPerDist));
    }
  }

  public double getRotations(){
    return m_AmpLift.getPosition().getValueAsDouble();
  }

  public double getPosition(){
    return getRotations()/Constants.AmpLiftRotPerDist;
  }

  public void calibratePos(double setPoint){
    m_AmpLift.setPosition(setPoint*Constants.AmpLiftRotPerDist);
  }

  public boolean atUpperLimit(){
    return false;
   // return !upperLimit.get();
  }

  public boolean atLowerLimit(){
    return !lowerLimit.get();
  }

  public void stayAtPosition(double AmpLiftKg) {
    //m_AmpLift.set(AmpLiftKg);
  }

  public void setBrakeLift () {
    m_AmpLift.setControl(AmpLiftbrake);


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (atUpperLimit()) {
      calibratePos(Constants.AmpLiftUpperLimitPos);
    } 
    if (atLowerLimit()) {
      calibratePos(Constants.AmpLiftLowerLimitPos);
  }

    if (getPosition() > 300){
      ampLiftAboveHandOff = true;
    }else{
      ampLiftAboveHandOff = false;
    }
    

  SmartDashboard.putNumber("AmpLift Current Rotations", getRotations());
  SmartDashboard.putNumber("AmpLift Current POS", getPosition());
  //SmartDashboard.putBoolean("AmpLift Upper Limit", atUpperLimit());
  SmartDashboard.putBoolean("AmpLift Lower Limit ", atLowerLimit());
  }
}
