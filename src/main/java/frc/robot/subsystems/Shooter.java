// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Shooter extends SubsystemBase {
 TalonFX m_ShooterTop = new TalonFX(17,"canivore");
 TalonFX m_ShooterBottom = new TalonFX(18,"canivore");
  TalonFX m_ShooterPitch = new TalonFX(15, "canivore");
  CANcoder CancoderPitch = new CANcoder(16, "canivore");
  double temp_target = 20;
  DigitalInput di_HandoffSensor = new DigitalInput(3);
  public double distanceToTarget = 1;
   private final VelocityVoltage VoltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
   private final PositionVoltage VoltagePosition = new PositionVoltage(0, 10, false, 0, 0, false, false, false);
   private final NeutralOut coast = new NeutralOut();
      private final StaticBrake brake = new StaticBrake();
   
  private Boolean ManualPitch;

  public static boolean autoPitchEnable = false;

  public Shooter(){
    ManualPitch =false;
    SmartDashboard.putNumber("Set Top RPM", 3000);
    SmartDashboard.putNumber("Set Bottom RPM", 3000);
    SmartDashboard.putNumber("target pitch", 20); //test
   
        TalonFXConfiguration configsShooter = new TalonFXConfiguration();
        
    SmartDashboard.putNumber("Auto Pitch P", Constants.autoPitch_P);
    SmartDashboard.putNumber("Auto Pitch I", Constants.autoPitch_I);
    SmartDashboard.putNumber("Auto Pitch D", Constants.autoPitch_D);

    SmartDashboard.putNumber("Auto Pitch kS",0);

    /* Voltage-based velocity requires  feed forward to account for the back-emf of the motor */
    configsShooter.Slot0.kP = Constants.AutoShooter_P; // An error of 1 rotation per second results in 2V output
    configsShooter.Slot0.kI = Constants.AutoShooter_I; // An error of 1 rotation per second increases output by 0.5V every second
    configsShooter.Slot0.kD = Constants.AutoShooter_D; // A change of 1 rotation per second squared results in 0.01 volts output
    configsShooter.Slot0.kV = 0.12; // Falcon 500 is  500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    SmartDashboard.putNumber("AutoShooter_P", Constants.AutoShooter_P);
    SmartDashboard.putNumber("AutoShooter_I", Constants.AutoShooter_I);
    SmartDashboard.putNumber("AutoShooter_D", Constants.AutoShooter_D);

    // Peak output of 8 volts
    configsShooter.Voltage.PeakForwardVoltage = 100;
    configsShooter.Voltage.PeakReverseVoltage = -100;

    TalonFXConfiguration configsPitch = new TalonFXConfiguration();
    configsPitch.Slot0.kP = Constants.autoPitch_P; // An error of 1 rotation per second results in 2V output
    configsPitch.Slot0.kI = Constants.autoPitch_I; // An error of 1 rotation per second increases output by 0.5V every second
    configsPitch.Slot0.kD = Constants.autoPitch_D; // A change of 1 rotation per second squared results in 0.01 volts output

    //configsPitch.Slot0.kS = Constants.autoPitch_ks;
    configsPitch.Voltage.PeakForwardVoltage = Constants.pitchMaxOutput;
    configsPitch.Voltage.PeakReverseVoltage = Constants.pitchMinOutput;


    // Peak output of 8 volts
  
    applyConfigs(m_ShooterTop, configsShooter, "m_ShooterTop");
    applyConfigs(m_ShooterBottom, configsShooter, "m_ShooterBottom");
    applyConfigs(m_ShooterPitch, configsPitch, "m_ShooterPitch");
    calibratePitch();
    
    m_ShooterBottom.setInverted(false);
    m_ShooterPitch.setInverted(true);
 
  }

  public boolean isHandOffSensorBlocked() {
    return di_HandoffSensor.get();
  }


  public void reapplyConfigs() {
    TalonFXConfiguration configsShooter = new TalonFXConfiguration();
    configsShooter.Slot0.kP = SmartDashboard.getNumber("AutoShooter_P", 0); // An error of 0.5 rotations results in 1.2 volts output
    configsShooter.Slot0.kI = SmartDashboard.getNumber("AutoShooter_I", 0);
    configsShooter.Slot0.kD = SmartDashboard.getNumber("AutoShooter_D", 0); // A change of 1 rotation per second results in 0.1 volts output
    applyConfigs(m_ShooterTop, configsShooter, "m_ShooterTop");
    applyConfigs(m_ShooterBottom, configsShooter, "m_ShooterBottom");
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


  public double getPitch(){
    return m_ShooterPitch.getPosition().getValueAsDouble() / Constants.pitchRotationsPerDegree;

  }

  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(CancoderPitch.getAbsolutePosition().getValue() + Constants.shooterPitchCancoderCal.getRotations());
  }

  public Rotation2d getAbsoluteCANcoder(){
    return Rotation2d.fromRotations(CancoderPitch.getAbsolutePosition().getValue());
    
    
  }

  public void calibratePitch(){
    m_ShooterPitch.setPosition(getCANcoder().getDegrees()*Constants.pitchRotationsPerDegree);  
  }

  public void setPitchPosition(double setAngle){ 
    if (Constants.pitchMaxAngle >= setAngle && setAngle >= Constants.pitchMinAngle) {
      m_ShooterPitch.setControl(VoltagePosition.withPosition(setAngle * Constants.pitchRotationsPerDegree));
    }
  }

   public void setPitchPercent(double setSpeed){
  // if(setSpeed < 0 && getPitch() < Constants.pitchMaxAngle){
  //   m_ShooterPitch.set(setSpeed);
  // }else if(setSpeed > 0 && getPitch() > Constants.pitchMinAngle){
    
  // }else{
  //   setPitchBrake();
  // }
  m_ShooterPitch.set(setSpeed);
    
   }
   public void setPitchBrake(){
     m_ShooterPitch.setControl(brake);
   }
 

public void setAutoPitchConstants(){
TalonFXConfiguration configsPitch = new TalonFXConfiguration();
    configsPitch.Slot0.kP = SmartDashboard.getNumber("Auto Pitch P", 0); // An error of 0.5 rotations results in 1.2 volts output
    configsPitch.Slot0.kI = SmartDashboard.getNumber("Auto Pitch I", 0);
    configsPitch.Slot0.kD = SmartDashboard.getNumber("Auto Pitch D", 0); // A change of 1 rotation per second results in 0.1 volts output
    configsPitch.Slot0.kS = SmartDashboard.getNumber("Auto Pitch kS", 0);
    configsPitch.Voltage.PeakForwardVoltage = Constants.pitchMaxOutput;
    configsPitch.Voltage.PeakReverseVoltage = Constants.pitchMinOutput;
    applyConfigs(m_ShooterPitch, configsPitch, "m_ShooerPitch");
}


 

  public void setPitchManualMode(boolean Manual){
    ManualPitch = Manual;
     m_ShooterPitch.setControl(brake);
  }
 
  public void coastPitch(){
    m_ShooterPitch.setControl(coast);
    
  }

  public boolean atSpeed(double bottomTargetSpeed, double topTargetSpeed) {
   double topCurrentSpeed = m_ShooterTop.getVelocity().getValueAsDouble()*60;
   double bottomCurrentSpeed = m_ShooterBottom.getVelocity().getValueAsDouble()*60;
   
    return(topCurrentSpeed > topTargetSpeed - Constants.targetSpeedTolerance
     && bottomCurrentSpeed > bottomTargetSpeed - Constants.targetSpeedTolerance);
  }

  public boolean atSpeedLob(double bottomTargetSpeed, double topTargetSpeed) {
   double topCurrentSpeed = m_ShooterTop.getVelocity().getValueAsDouble()*60;
   double bottomCurrentSpeed = m_ShooterBottom.getVelocity().getValueAsDouble()*60;
   
    return(topCurrentSpeed > topTargetSpeed - 200
     && bottomCurrentSpeed > bottomTargetSpeed - 200);
  }


  public void setRPM(double Top_RPM, double Bottom_RPM) {
    m_ShooterTop.setControl(VoltageVelocity.withVelocity(Top_RPM/60));
    m_ShooterBottom.setControl(VoltageVelocity.withVelocity(Bottom_RPM/60));

  }
  public void coast() {
    m_ShooterTop.setControl(coast);
    m_ShooterBottom.setControl(coast);
  }

  public void setPercent(double shooterPercent) {
   if (!AmpLift.ampLiftAboveHandOff){
     m_ShooterTop.set(shooterPercent);
     m_ShooterBottom.set(shooterPercent);
   }
    
  }
    //insert quadratic formula here
  public double shooterPitchFromDistance(double DistanceToSpeaker) {
    double pitch = Constants.ShooterPitchCalc_A * Math.pow(DistanceToSpeaker, 3) 
                    + Constants.ShooterPitchCalc_B * Math.pow(DistanceToSpeaker, 2) 
                    + Constants.ShooterPitchCalc_C * DistanceToSpeaker
                    + Constants.ShooterPitchCalc_D;
    

    return (pitch);
  }

 
    public boolean pitchAtTarget(double targetPitch){
      double currentPitch = getPitch();
      double error = currentPitch - targetPitch; 

      return(Math.abs(error)< Constants.pitch_tol);
  }

    public boolean pitchAtTargetLob(double targetPitch){
      double currentPitch = getPitch();
      double error = currentPitch - targetPitch; 

      return(Math.abs(error)< 2);
  }




  


  @Override
  public void periodic() {

    if (getCANcoder().getDegrees() > Constants.pitchMaxAngle || getCANcoder().getDegrees() < Constants.pitchMinAngle){
      calibratePitch();
    }

    // if (distanceToTarget < 6) {
    //   temp_target = shooterPitchFromDistance(distanceToTarget);
    // }
   

    if (AmpLift.ampLiftAboveHandOff){
      coast();
    }
    if (autoPitchEnable) {
      setPitchPosition(shooterPitchFromDistance(RobotContainer.s_Swerve.getDistanceToSpeaker(false)));
    }

    SmartDashboard.putBoolean("shooter at RPM", atSpeed(Constants.ShooterSpeed, Constants.ShooterSpeed));
    SmartDashboard.putBoolean("Pitch at Pos", pitchAtTarget(temp_target));


    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Bottom Actual RPM", m_ShooterBottom.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Shooter Top Actual RPM", m_ShooterTop.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Shooter Pitch Angle", getPitch());
    SmartDashboard.putNumber("CANcoder Absolute Pitch Angle", getAbsoluteCANcoder().getDegrees());
    SmartDashboard.putNumber("Shooter Pitch Rotations", m_ShooterPitch.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("CANcoder Pitch Angle", getCANcoder().getDegrees());
    SmartDashboard.putBoolean("Hand-Off Sensor", isHandOffSensorBlocked());
    SmartDashboard.putNumber("Calculated Target Pitch", shooterPitchFromDistance(distanceToTarget));
    
       //resetToAbsolute();
  }
}
