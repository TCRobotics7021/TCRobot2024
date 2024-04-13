package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShootNoteIntoSpeaker extends Command {
  /** Creates a new ShootNoteIntoSpeaker. */
  boolean finished;
  boolean idleAfter;
  double targetAngle;
  double rotationVal;
  double targetPitch;
  double DistanceToSpeaker;
  double angleAdjust;
  Timer startdelay = new Timer();
  Timer stopdelay = new Timer();

  public AutoShootNoteIntoSpeaker(double angleAdjust, boolean idleAfter) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.idleAfter = idleAfter;
    this.angleAdjust = angleAdjust;
    addRequirements(RobotContainer.s_Swerve, RobotContainer.s_Intake, RobotContainer.s_Shooter, RobotContainer.s_AmpLift);
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
    RobotContainer.s_Shooter.calibratePitch();
    Shooter.autoPitchEnable = false;
    Swerve.overrideRotation = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.s_AmpLift.setPercentLift(-Constants.AmpLiftJogPercent);

   //Aiming
  
    targetAngle = RobotContainer.s_Swerve.getAngleToSpeaker(false);
    rotationVal = RobotContainer.s_Swerve.getRotationOutput(targetAngle);
    
    /* Drive */
         var alliance = DriverStation.getAlliance();
         //SmartDashboard.putNumber("Swerve Rotation Value", rotationVal);
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity,
            true,
            true
        );
        } else {
             RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );  
        }
     
    //Shooting
    RobotContainer.s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed); 

    //Pitch
    
    DistanceToSpeaker =RobotContainer.s_Swerve.getDistanceToSpeaker(false);
    targetPitch = RobotContainer.s_Shooter.shooterPitchFromDistance(DistanceToSpeaker) + angleAdjust;
   
    RobotContainer.s_Shooter.setPitchPosition(targetPitch);   
//SmartDashboard.putBoolean("Aimed at Speaker", aimedAtSpeaker());
    if ((RobotContainer.s_Shooter.atSpeed(Constants.ShooterSpeed,Constants.ShooterSpeed) 
          && RobotContainer.s_Swerve.aimedAtSpeaker(targetAngle)
          && RobotContainer.s_Shooter.pitchAtTarget(targetPitch))){
          
        startdelay.start();
   }else{
    //startdelay.reset();
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
           if (idleAfter == true){
              RobotContainer.s_Shooter.setRPM(Constants.IdleSpeed, Constants.IdleSpeed);
           } 
           RobotContainer.s_AmpLift.setBrakeLift();
           
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
