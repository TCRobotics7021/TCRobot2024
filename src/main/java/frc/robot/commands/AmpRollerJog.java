// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AmpRollerJog extends Command {
double setPercent;
 boolean finished;
  /** Creates a new JogAndSetPOS. */
  public AmpRollerJog(double setPercent) {
    addRequirements(RobotContainer.s_AmpLift);
    this.setPercent = setPercent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_AmpLift.setPercentRollers(setPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_AmpLift.setPercentRollers(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
