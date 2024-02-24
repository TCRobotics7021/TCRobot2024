// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Limelight extends SubsystemBase {
  NetworkTable limeLightTable;


  /** Creates a new Limelight. */
  public Limelight() {
    limeLightTable = NetworkTableInstance.getDefault().getTable("limelight-mitch"); //name might be just "mitch"
  }



  public double targetX(){
    return limeLightTable.getEntry("tx").getDouble(0);
  }

  public double targetY(){
    return limeLightTable.getEntry("ty").getDouble(0);
  }

  public double targetA(){
    return limeLightTable.getEntry("ta").getDouble(0);
  }

  public boolean isNote(){
    return (limeLightTable.getEntry("tv").getDouble(0) == 1);
  }

  public void setPipeline(int pipelineNumber){
    limeLightTable.getEntry("pipeline").setNumber(pipelineNumber);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Note Vision Target X", targetX());
    SmartDashboard.putNumber("Note Vision Target Y", targetY());
    SmartDashboard.putBoolean("Vision Is Note Present?", isNote());
  }
}
