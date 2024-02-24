package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TestShooting extends Command {
  /** Creates a new ShootNoteIntoSpeaker. */
  
  
  double targetAngle;
  double rotationVal;



  public TestShooting() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Intake, RobotContainer.s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topspeed = SmartDashboard.getNumber("Set Top RPM", 0);
    double botspeed = SmartDashboard.getNumber("Set Bottom RPM", 0);
    //Shooting
    RobotContainer.s_Shooter.setRPM(topspeed, botspeed); 
    if (RobotContainer.s_Shooter.atSpeed(topspeed,botspeed)){
      RobotContainer.s_Intake.setPercent(Constants.feedPercent);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
           RobotContainer.s_Intake.coast();
           RobotContainer.s_Shooter.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
