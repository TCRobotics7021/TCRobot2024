// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class PitchSetPOS extends Command {
  /** Creates a new PitchSetPOS. */
  double setPitch;
  boolean finished;
  public PitchSetPOS(double setPitch) {
    // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(RobotContainer.s_Shooter);
     this.setPitch = setPitch;
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    RobotContainer.s_Shooter.setPitch(setPitch);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   finished = RobotContainer.s_Shooter.pitchAtTarget(setPitch);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
