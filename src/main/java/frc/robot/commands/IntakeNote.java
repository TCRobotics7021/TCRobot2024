// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */

  Timer delay = new Timer();

  boolean finished;
  public IntakeNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Intake,RobotContainer.s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay.reset();
    delay.stop();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Intake.sensorIsBlocked() == true){
      delay.start();
      
    } else {
      delay.reset();
      RobotContainer.s_Intake.setPercent(Constants.intakePercent);
    }

    if(delay.get()>.05){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Intake.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
  