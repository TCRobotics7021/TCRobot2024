//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Lift. */
  TalonFX m_Climber = new TalonFX(21, "canivore");
  double liftKg = Constants.ClimberKg;
  private final StaticBrake brake = new StaticBrake();
  private final PositionVoltage VoltagePosition = new PositionVoltage(0, 10, false, 0, 0, false, false, false);
  
  DigitalInput upperLimit = new DigitalInput(2);
  DigitalInput lowerLimit = new DigitalInput(3);

  public Climber() {
    SmartDashboard.putNumber("Climberconfigs_P", Constants.Climberconfigs_P); // An error of 0.5 rotations results in 1.2 volts output
    SmartDashboard.putNumber("Climberconfigs_I", Constants.Climberconfigs_I);
    SmartDashboard.putNumber("Climberconfigs_D", Constants.Climberconfigs_D); // A change of 1 rotation per second results in 0.1 volts output
    SmartDashboard.putNumber("Climberconfigs_kG", Constants.Climberconfigs_kG);
    SmartDashboard.putNumber("Climberconfigs_kS", Constants.Climberconfigs_kS);
    m_Climber.setInverted(false);
    TalonFXConfiguration configsClimber = new TalonFXConfiguration();
    configsClimber.Slot0.kP = Constants.Climberconfigs_P; // An error of 1 rotation per second results in 2V output
    configsClimber.Slot0.kI = Constants.Climberconfigs_I; // An error of 1 rotation per second increases output by 0.5V every second
    configsClimber.Slot0.kD = Constants.Climberconfigs_D; // A change of 1 rotation per second squared results in 0.01 volts output
    configsClimber.Slot0.kG = Constants.Climberconfigs_kG;
    configsClimber.Slot0.kS = Constants.Climberconfigs_kS;
    configsClimber.Voltage.PeakForwardVoltage = 2;
    configsClimber.Voltage.PeakReverseVoltage = -2;
    
    applyConfigs(m_Climber, configsClimber, " m_Climber");
  }
public void reapplyConfigs() {
    TalonFXConfiguration configsClimber = new TalonFXConfiguration();
    configsClimber.Slot0.kP = SmartDashboard.getNumber("Climberconfigs_P", Constants.Climberconfigs_P); // An error of 0.5 rotations results in 1.2 volts output
    configsClimber.Slot0.kI = SmartDashboard.getNumber("Climberconfigs_I", Constants.Climberconfigs_I);
    configsClimber.Slot0.kD = SmartDashboard.getNumber("Climberconfigs_D", Constants.Climberconfigs_D); // A change of 1 rotation per second results in 0.1 volts output
    configsClimber.Slot0.kG = SmartDashboard.getNumber("Climberconfigs_kG", Constants.Climberconfigs_kG);
    configsClimber.Slot0.kS = SmartDashboard.getNumber("Climberconfigs_kS", Constants.Climberconfigs_kS);
   
    applyConfigs(m_Climber, configsClimber, "mClimberconfigs");

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

  public void setPercent(double setPercent) {
    if (setPercent < 0 && (atLowerLimit()||getPosition()<Constants.ClimberMINPos)) {
      setBrake();
    } else if (setPercent > 0 && (atUpperLimit()||getPosition()>Constants.ClimberMAXPos)){
      setBrake();
    } else {
      m_Climber.set(setPercent);
    }
  }

  public void setPosition(double setPosition){
    
    if (Constants.ClimberMAXPos > setPosition && setPosition > Constants.ClimberMINPos) {
      m_Climber.setControl(VoltagePosition.withPosition(setPosition * Constants.ClimberConversionFactor));
    }
  }

  public double getRotations(){
    return m_Climber.getPosition().getValueAsDouble();
  }

  public double getPosition(){
    return getRotations()/Constants.ClimberConversionFactor;
  }

  public void calibratePos(double setPoint){
    m_Climber.setPosition(setPoint*Constants.ClimberConversionFactor);
  }

  public boolean atUpperLimit(){
    
    return !upperLimit.get();
  }

  public boolean atLowerLimit(){
    return !lowerLimit.get();
  }

  public void stayAtPosition(double ClimberKg) {
    //m_Climber.set(ClimberKg);
  }

  public void setBrake() {
    m_Climber.setControl(brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (atUpperLimit()) {
      calibratePos(Constants.ClimberUpperLimitPos);
    } 
    if (atLowerLimit()) {
      calibratePos(Constants.ClimberLowerLimitPos);
  }
  SmartDashboard.putNumber("Climber Current Rotations", getRotations());
  SmartDashboard.putNumber("Climber Current POS", getPosition());
  SmartDashboard.putBoolean("Climber Upper Limit", atUpperLimit());
  SmartDashboard.putBoolean("Climber Lower Limit ", atLowerLimit());
}

}