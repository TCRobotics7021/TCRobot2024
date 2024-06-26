// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class AmpLiftHome extends Command {
  boolean finished;
  public AmpLiftHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_AmpLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_AmpLift.calibratePos(Constants.AmpLiftMAXPos);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_AmpLift.setPercentLift(-Constants.AmpLiftJogPercent);
    if (RobotContainer.s_AmpLift.atLowerLimit()){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_AmpLift.setBrakeLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
