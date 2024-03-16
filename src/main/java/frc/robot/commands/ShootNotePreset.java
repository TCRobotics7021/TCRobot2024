// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShootNotePreset extends Command {
 boolean finished;
  double targetAngle;
  double rotationVal;
  double targetPitch;
  double DistanceToSpeaker;
  double pitch;
  double RPM;
  double redRot;
  double blueRot;
  Timer startdelay = new Timer();
  Timer stopdelay = new Timer();




  public ShootNotePreset(double pitch, double RPM, double redRot, double blueRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve, RobotContainer.s_Intake, RobotContainer.s_Shooter);
    this.pitch = pitch;
    this.RPM = RPM;
    this.redRot = redRot;
    this.blueRot = blueRot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startdelay.reset();
    startdelay.stop();
    stopdelay.reset();
    stopdelay.stop();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   //Aiming
      var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Red) {
        targetAngle = redRot;
      } else {
        targetAngle = blueRot;
      }
      rotationVal = RobotContainer.s_Swerve.getRotationOutput(targetAngle);
   
    
     RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
           //og is true
           true,
            true
        );


    //Pitch

    RobotContainer.s_Shooter.setPitch(pitch);
    RobotContainer.s_Shooter.setRPM(RPM, RPM);

    if ((RobotContainer.s_Shooter.atSpeed(RPM, RPM) 
          && (RobotContainer.s_Swerve.atRotation(targetAngle))
          && (RobotContainer.s_Shooter.pitchAtTarget(pitch))
          )){
            
        startdelay.start();
   }else{
    startdelay.reset();
   }

   if(startdelay.get() > .00){
    RobotContainer.s_Intake.setPercent(Constants.feedPercent);
   }

   if(RobotContainer.s_Intake.sensorIsBlocked() == false){
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
           //RobotContainer.s_Shooter.coast();
           RobotContainer.s_Shooter.setRPM(Constants.IdleSpeed, Constants.IdleSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
