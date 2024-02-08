// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootSpeed extends Command {
  /** Creates a new ShootSpeed. */
  public ShootSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     RobotContainer.s_Shooter.setRPM(SmartDashboard.getNumber("Set Top RPM", 0)/60, SmartDashboard.getNumber("Set Bottom RPM", 0)/60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Shooter.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
