// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class EjectIntake extends Command {
  /** Creates a new EjectIntake. */
  public EjectIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(RobotContainer.s_Intake, RobotContainer.s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_Intake.setPercent(-Constants.intakePercent);
    RobotContainer.s_Shooter.setPercent(-.3);
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
