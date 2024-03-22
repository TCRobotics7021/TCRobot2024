// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonVision;

public class ShootNoteIntoSpeaker extends Command {
  /** Creates a new ShootNoteIntoSpeaker. */
  
  boolean finished;
  double targetAngle;
  double rotationVal;
  double targetPitch;
  double DistanceToSpeaker;
  Timer startdelay = new Timer();
  Timer stopdelay = new Timer();
  private BooleanSupplier AprilTagToggle;
  private BooleanSupplier ManualAim;
  private BooleanSupplier ManualPitch;
  private BooleanSupplier RobotCentric;

  public ShootNoteIntoSpeaker(BooleanSupplier AprilTagToggle, BooleanSupplier ManualAim, BooleanSupplier ManualPitch, BooleanSupplier RobotCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.AprilTagToggle = AprilTagToggle;
    this.ManualAim = ManualAim;  
    this.ManualPitch = ManualPitch;
    this.RobotCentric = RobotCentric;
    addRequirements(RobotContainer.s_Swerve, RobotContainer.s_Intake, RobotContainer.s_Shooter, RobotContainer.s_PhotonVision, RobotContainer.s_AmpLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startdelay.reset();
    startdelay.stop();
    stopdelay.reset();
    stopdelay.stop();
    finished = false;
    RobotContainer.s_Swerve.resetAutoRotatePID();
    RobotContainer.s_Shooter.resetAutoPitchPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.s_AmpLift.setPercentLift(-Constants.AmpLiftJogPercent);

   //Aiming
    if (!ManualAim.getAsBoolean() && !RobotCentric.getAsBoolean()){
      targetAngle = RobotContainer.s_Swerve.getAngleToSpeaker();
      rotationVal = RobotContainer.s_Swerve.getRotationOutput(targetAngle);
    } else {
      rotationVal = 0;
    }
   
    
     RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
           //og is true
           true,
            true
        );
    //Shooting
    RobotContainer.s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed); 

    //Pitch
    if (!ManualAim.getAsBoolean() && !RobotCentric.getAsBoolean()){
    DistanceToSpeaker =RobotContainer.s_Swerve.getDistanceToSpeaker();
    targetPitch = RobotContainer.s_Shooter.shooterPitchFromDistance(DistanceToSpeaker);
    } else {
      targetPitch = Constants.defaultPitch;
    }
    RobotContainer.s_Shooter.setPitch(targetPitch);   

    if ((RobotContainer.s_Shooter.atSpeed(Constants.ShooterSpeed,Constants.ShooterSpeed) 
          && (RobotContainer.s_Swerve.aimedAtSpeaker() || ManualAim.getAsBoolean() || RobotCentric.getAsBoolean())
          && (RobotContainer.s_Shooter.pitchAtTarget(targetPitch) || ManualPitch.getAsBoolean()))
          && ((PhotonVision.AprilTagVisible && AprilTagToggle.getAsBoolean()) || !AprilTagToggle.getAsBoolean())){
            
        startdelay.start();
   }else{
    startdelay.reset();
   }

   if(startdelay.get() > .25){
    RobotContainer.s_Intake.setPercent(Constants.feedPercent);
   }

   if(RobotContainer.s_Intake.sensorIsBlocked() == false){
      stopdelay.start();
   }else{
      stopdelay.reset();
   }

   if(stopdelay.get() > .25){
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
           RobotContainer.s_AmpLift.setBrakeLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
