// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class FeedNoteFromIntakeToAmpLift extends Command {
  /** Creates a new FeedNoteToAmpLift. */
  boolean finished;

  public FeedNoteFromIntakeToAmpLift() {

    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(RobotContainer.s_Intake);
      addRequirements(RobotContainer.s_Shooter);
      addRequirements(RobotContainer.s_AmpLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_Intake.setPercent(Constants.IntakeHandOffPercent);
    RobotContainer.s_Shooter.setPercent(Constants.ShooterHandOffPercent);
    RobotContainer.s_AmpLift.setPercentRollers(Constants.AmpLiftHandOffPercent);
    if (!RobotContainer.s_Intake.sensorIsBlocked() && !RobotContainer.s_Shooter.isHandOffSensorBlocked()){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Intake.coast();
    RobotContainer.s_Shooter.coast();
    RobotContainer.s_AmpLift.setPercentRollers(0);    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
