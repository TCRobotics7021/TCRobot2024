// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class DriveForward extends Command {

  /** Creates a new DriveFoward. */
  public DriveForward() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        RobotContainer.s_Swerve.drive(
            new Translation2d(.75, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            false
        );  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
