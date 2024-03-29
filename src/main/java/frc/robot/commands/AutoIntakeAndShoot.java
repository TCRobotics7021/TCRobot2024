// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoIntakeAndShoot extends Command {
  /** Creates a new AutoIntakeAndShoot. */
  boolean finished;
  boolean shotstarted;
  double xError;
  double yError;
  double calcTranslation;
  double calcStrafe;
  Timer t_delay = new Timer();

  boolean noteHasBeenIntaked;
 
  Timer aimingTimeout = new Timer();
  double targetAngle;
  double calcRotation;
  double targetPitch;
  double DistanceToSpeaker;
  Timer stopdelay = new Timer();

  public AutoIntakeAndShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Limelight, RobotContainer.s_Swerve, RobotContainer.s_Intake, RobotContainer.s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    t_delay.reset();
    t_delay.stop();
    aimingTimeout.reset();
    aimingTimeout.stop();
    noteHasBeenIntaked = false;
    shotstarted = false;

  
    stopdelay.reset();
    stopdelay.stop();
    RobotContainer.s_Swerve.resetAutoRotatePID();
    RobotContainer.s_Shooter.resetAutoPitchPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Intake.sensorIsBlocked() ==true){
      noteHasBeenIntaked = true;
      aimingTimeout.start();
    }

    if ( t_delay.get() > 3 && noteHasBeenIntaked==false) {
      finished = true;
    }
    xError = RobotContainer.s_Limelight.targetX();
    yError = RobotContainer.s_Limelight.targetY();

   



    if (yError >= 5 && RobotContainer.s_Limelight.isNote()) {

      calcStrafe = Constants.strafeP * xError;
      calcTranslation = Math.min(Constants.translatePStrafeStrat * Math.abs(xError) - .25,0); // changed .5 to .25
      t_delay.reset();
      t_delay.stop();
    } else if(RobotContainer.s_Limelight.isNote()) {
      calcStrafe = 0; 
      calcTranslation = -.25;
      t_delay.start();
    } else{
      calcStrafe = 0;
      calcTranslation = 0;
      t_delay.start();
    }

    
    SmartDashboard.putNumber("calcTranslation", calcTranslation);
    SmartDashboard.putNumber("calcStrafe", calcStrafe);

    //Aiming
    targetAngle = RobotContainer.s_Swerve.getAngleToSpeaker(false);//look at later
    calcRotation = RobotContainer.s_Swerve.getRotationOutput(targetAngle);
    
    if(RobotContainer.s_Limelight.isNote()){
     calcRotation = calcRotation*0;
    }

     RobotContainer.s_Swerve.drive(
            new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
           calcRotation * Constants.Swerve.maxAngularVelocity, 
           //og is true
           false,
            true
        );
    //Shooting
    RobotContainer.s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed); 

    //Pitch
    DistanceToSpeaker =RobotContainer.s_Swerve.getDistanceToSpeaker(false);
    targetPitch = RobotContainer.s_Shooter.shooterPitchFromDistance(DistanceToSpeaker);
    RobotContainer.s_Shooter.setPitch(targetPitch);   

   if(noteHasBeenIntaked==false){
    if(RobotContainer.s_Limelight.isNote()){
    RobotContainer.s_Intake.setPercent(Constants.intakePercent);
    }
   }else if (noteHasBeenIntaked==true 
          && RobotContainer.s_Shooter.atSpeed(Constants.ShooterSpeed,Constants.ShooterSpeed ) 
          && RobotContainer.s_Swerve.aimedAtSpeaker(targetAngle) 
          && RobotContainer.s_Shooter.pitchAtTarget(targetPitch) ) {
            shotstarted = true;
     RobotContainer.s_Intake.setPercent(Constants.feedPercent);
   }else if(aimingTimeout.get()>2){
      RobotContainer.s_Intake.setPercent(Constants.feedPercent);
       shotstarted = true;
   }else if(!shotstarted){
     RobotContainer.s_Intake.coast();
     //RobotContainer.s_Shooter.setRPM(Constants.IdleSpeed, Constants.IdleSpeed);
   }

   if(RobotContainer.s_Intake.sensorIsBlocked() == false && noteHasBeenIntaked){
      stopdelay.start();
   }else{
      stopdelay.reset();
   }

   if(stopdelay.get() > .5){
    finished = true;
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
     true,
      true
  );
     RobotContainer.s_Intake.coast();
     RobotContainer.s_Shooter.setRPM(Constants.IdleSpeed, Constants.IdleSpeed);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
