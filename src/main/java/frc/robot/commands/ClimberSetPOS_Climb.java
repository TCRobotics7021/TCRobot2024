// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSetPOS_Climb extends Command {
   double setPosition;
  boolean finished;
  
  public ClimberSetPOS_Climb(double setPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setPosition = setPosition;
    addRequirements(RobotContainer.s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_Climber.setPosition(setPosition);

  if (Math.abs(setPosition - RobotContainer.s_Climber.getPosition()) < Constants.ClimberTolerance){
    finished = true;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.s_Climber.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
