// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
 TalonFX m_ShooterTop = new TalonFX(17,"canivore");
 TalonFX m_ShooterBottom = new TalonFX(18,"canivore");
  TalonFX m_ShooterPitch = new TalonFX(15, "canivore");
  CANcoder CancoderPitch = new CANcoder(16, "canivore");
  double temp_target = 20;
  public double distanceToTarget = 1;
   private final VelocityVoltage VoltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
   private final NeutralOut coast = new NeutralOut();
      private final StaticBrake brake = new StaticBrake();
   private final PositionVoltage VoltagePosition = new PositionVoltage(0, 10, false, 0, 0, false, false, false);


  public Shooter(){
    SmartDashboard.putNumber("Set Top RPM", 3000);
    SmartDashboard.putNumber("Set Bottom RPM", 3000);
    SmartDashboard.putNumber("target pitch", 20); //test
   
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
    //SoftwareLimitSwitchConfigs configsLimitSwitchPitch = new SoftwareLimitSwitchConfigs();
    configsPitch.Slot0.kP = 5; // An error of 0.5 rotations results in 1.2 volts output
    configsPitch.Slot0.kI = 0;
    configsPitch.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output

    SmartDashboard.putNumber("Shooter Pitch P", configsPitch.Slot0.kP);
    SmartDashboard.putNumber("Shooter Pitch I", configsPitch.Slot0.kI);
    SmartDashboard.putNumber("Shooter Pitch D", configsPitch.Slot0.kD);


    // Peak output of 8 volts
    configsPitch.Voltage.PeakForwardVoltage = 8;
    configsPitch.Voltage.PeakReverseVoltage = -8;
    

    configsPitch.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    configsPitch.TorqueCurrent.PeakReverseTorqueCurrent = 130;
  
    configsPitch.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    configsPitch.Feedback.FeedbackRemoteSensorID = CancoderPitch.getDeviceID();
    configsPitch.Feedback.SensorToMechanismRatio = 1;
    // configsLimitSwitchPitch.ForwardSoftLimitEnable = true;
    // configsLimitSwitchPitch.ReverseSoftLimitEnable = true;
    // configsLimitSwitchPitch.ForwardSoftLimitThreshold = Constants.pitchMaxAngle.getRotations();
    // configsLimitSwitchPitch.ReverseSoftLimitThreshold = Constants.pitchMinAngle.getRotations();
  
    applyConfigs(m_ShooterTop, configsShooter, "m_ShooterTop");
    applyConfigs(m_ShooterBottom, configsShooter, "m_ShooterBottom");
    applyConfigs(m_ShooterPitch, configsPitch, "m_ShooterPitch");
    //applyConfigs(m_ShooterPitch, configsLimitSwitchPitch, "m_ShooterPitch (SoftLimitSwitch)");
    resetToAbsolute();
    
    m_ShooterBottom.setInverted(false);
    m_ShooterPitch.setInverted(true);
 
  }


  public void reapplyConfigs() {
    TalonFXConfiguration configsPitch = new TalonFXConfiguration();
    configsPitch.Slot0.kP = SmartDashboard.getNumber("Shooter Pitch P", 0); // An error of 0.5 rotations results in 1.2 volts output
    configsPitch.Slot0.kI = SmartDashboard.getNumber("Shooter Pitch I", 0);
    configsPitch.Slot0.kD = SmartDashboard.getNumber("Shooter Pitch D", 0); // A change of 1 rotation per second results in 0.1 volts output
    applyConfigs(m_ShooterPitch, configsPitch, "m_ShooterPitch");
    
    
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


  public Rotation2d getPitch(){
    //return (Rotation2d.fromRotations(CancoderPitch.getAbsolutePosition().getValueAsDouble()).getDegrees() + Constants.shooterPitchCancoderCal);
    return Rotation2d.fromRotations(m_ShooterPitch.getPosition().getValue() + Constants.shooterPitchCancoderCal.getRotations());
  
  }

  
  public void setPitch(double targetPitch){
    temp_target = targetPitch;
    // double rot_pos = Rotation2d.fromDegrees(targetPitch).getRotations() - Constants.shooterPitchCancoderCal.getRotations();
    //  m_ShooterPitch.setControl(VoltagePosition.withPosition(rot_pos));
    //  SmartDashboard.putNumber("Pitch T Rot",Rotation2d.fromDegrees(targetPitch).getRotations());
    //  SmartDashboard.putNumber("Pitch Cal Rot",Constants.shooterPitchCancoderCal.getRotations());
    //  SmartDashboard.putNumber("Pitch Total Target Rot",rot_pos);
   }
   public void pitchPcontroller(){
        if(temp_target > Constants.pitchMaxAngle){
          temp_target = Constants.pitchMaxAngle;
        }
        if(temp_target < Constants.pitchMinAngle){
          temp_target = Constants.pitchMinAngle;
        }
        double error = temp_target - getPitch().getDegrees();
        double output = error * .05;//make constant

        
        if(Math.abs(error)< Constants.robotAngle_tol){
          m_ShooterPitch.setControl(brake);
        }
        m_ShooterPitch.set(output);
   }


  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(CancoderPitch.getAbsolutePosition().getValue());
    
    
  }
  public void resetToAbsolute(){
    // StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    // double absolutePosition = getCANcoder().getRotations() - Constants.shooterPitchCancoderCal.getRotations();
    // status1 = m_ShooterPitch.setPosition(absolutePosition);
    // if(!status1.isOK()) {
    //   System.out.println("Could not apply pitch position " + ":" + status1.toString());
    // } else {
    //   System.out.println("Configs successfully applied pitch position ");
    // }
    
    //CancoderPitch.setPosition(Constants.shooterPitchCancoderCal.getRotations());
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


  public void setRPM(double Top_RPM, double Bottom_RPM) {
    m_ShooterTop.setControl(VoltageVelocity.withVelocity(Top_RPM/60));
    m_ShooterBottom.setControl(VoltageVelocity.withVelocity(Bottom_RPM/60));

  }
  public void coast() {
    m_ShooterTop.setControl(coast);
    m_ShooterBottom.setControl(coast);
  }

  public void setPercent(double shooterPercent) {
    m_ShooterTop.set(shooterPercent);
    m_ShooterBottom.set(-shooterPercent);
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
      double currentPitch = getPitch().getDegrees();
      double error = currentPitch - targetPitch; 

      return(Math.abs(error)< Constants.pitch_tol);
  }
  


  @Override
  public void periodic() {

    if (distanceToTarget < 6) {
      temp_target = shooterPitchFromDistance(distanceToTarget);
    }
    pitchPcontroller();   

    SmartDashboard.putBoolean("shooter at RPM", atSpeed(Constants.ShooterSpeed, Constants.ShooterSpeed));
    SmartDashboard.putBoolean("Pitch at Pos", pitchAtTarget(temp_target));


    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Bottom Actual RPM", m_ShooterBottom.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Shooter Top Actual RPM", m_ShooterTop.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Shooter Pitch Angle", getPitch().getDegrees());
    SmartDashboard.putNumber("Shooter Absolute Pitch Angle", getCANcoder().getDegrees());
       //resetToAbsolute();
  }
}
