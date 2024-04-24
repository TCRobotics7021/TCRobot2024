// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class AutoNotePickUpRotate extends Command {
  boolean finished;
  double xError;
  double yError;
  double calcRotation;
  double calcTranslation;
  double calcStrafe;
  Timer t_delay = new Timer();

  /** Creates a new AutoNotePickUpRotate. */
  public AutoNotePickUpRotate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Limelight, RobotContainer.s_Swerve, RobotContainer.s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    t_delay.reset();
    t_delay.stop(); 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (RobotContainer.s_Intake.sensorIsBlocked() || t_delay.get() > 3) {
      finished = true;
    }
    xError = RobotContainer.s_Limelight.targetX();
    yError = RobotContainer.s_Limelight.targetY();

    if (RobotContainer.s_Limelight.isNote()){
      RobotContainer.s_Intake.setPercent(Constants.intakePercent);
    }



    if (yError >= 5 && RobotContainer.s_Limelight.isNote()) {

      calcRotation = -Constants.rotateP * xError;
      calcTranslation = Constants.translatePRotateStrat * Math.abs(xError) - .3; // changed -.5 to -.4
      t_delay.reset();
      t_delay.stop();
    } else if(RobotContainer.s_Limelight.isNote()) {
      calcRotation = 0;
      calcTranslation = -.15; // changed from .25 to .15
      t_delay.start();
    } else{
      calcRotation = 0;
      calcTranslation = 0;
      t_delay.start();
    }

    RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, 0).times(Constants.Swerve.maxSpeed), 
      calcRotation * Constants.Swerve.maxAngularVelocity, 
     //og is true
     false,
      true
  );
    SmartDashboard.putNumber("calcTranslation", calcTranslation);
    SmartDashboard.putNumber("calcRotation", calcRotation);
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
           RobotContainer.s_Intake.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
