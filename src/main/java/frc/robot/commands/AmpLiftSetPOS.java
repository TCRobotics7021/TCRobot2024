// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AmpLiftSetPOS extends Command {
  /** Creates a new AmpLiftSetPOS. */
  double setPosition;
  boolean finished;

  public AmpLiftSetPOS(double setPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_AmpLift);
    this.setPosition = setPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_AmpLift.setPosition(setPosition);

    if (Math.abs(setPosition - RobotContainer.s_AmpLift.getPosition()) < Constants.AmpLiftTolerance){
    finished = true;
  }
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
