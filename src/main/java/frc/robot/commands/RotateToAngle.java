// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RotateToAngle extends Command {
  /** Creates a new RotateToAngle. */
  double targetAngle;
  double rotationVal;

  public RotateToAngle(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    addRequirements(RobotContainer.s_Swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotationVal = RobotContainer.s_Swerve.getRotationOutput(targetAngle);
    
     RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
           true,
            true
        );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
           true,
            true
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
