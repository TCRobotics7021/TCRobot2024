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

public class ShooterCalibration extends Command {
  /** Creates a new ShootNoteIntoSpeaker. */
  
  
  double targetAngle;
  double rotationVal;
  double targetPitch;
  double DistanceToSpeaker;
    Timer delay = new Timer();

  public ShooterCalibration() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve, RobotContainer.s_Intake, RobotContainer.s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay.reset();
    delay.stop();
    RobotContainer.s_Shooter.calibratePitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Shooting
    RobotContainer.s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed); 

    //Pitch test
    targetPitch = SmartDashboard.getNumber("target pitch", 20);
    RobotContainer.s_Shooter.setPitchPosition(targetPitch);   

    if (RobotContainer.s_Shooter.atSpeed(Constants.ShooterSpeed,Constants.ShooterSpeed )
          && RobotContainer.s_Shooter.pitchAtTarget(targetPitch)){
            delay.start();
        
   }else{
    delay.reset();
   }
   if(delay.get() > 1){
    RobotContainer.s_Intake.setPercent(Constants.feedPercent);
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
           RobotContainer.s_Shooter.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
