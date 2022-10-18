// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc2016.commands;

import org.usfirst.frc2016.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grip_FindBalls extends Command {
  public Grip_FindBalls() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrainSRX);
  }

  private double STEER_K; // How hard to steer towards target
  private double DRIVE_K; // How hard to drive forward towards target
  private double DESIRED_TARGET_AREA; // Area of target when robot reaches wall
  private double MAX_SPEED; // Speed limit

  public double steerCommand = 0.0;
  public double driveCommand = 0.0;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelight.setDrive(0.4, 0.2, 100, 0.3);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    update();
    Robot.driveTrainSRX.arcadeDrive(-this.driveCommand, this.steerCommand);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public void update(){
    NetworkTable grip = NetworkTableInstance.getDefault().getTable("GRIP/BallLocation");
     NetworkTableEntry xEntry = grip.getEntry("x");
     //NetworkTableEntry yEntry = grip.getEntry("y");
     NetworkTableEntry sizeEntry = grip.getEntry("size");

     if (gripBallFound(sizeEntry)) { // If target is acquired
      
     double tx = xEntry.getDouble(0); //xEntry.getDoubleArray(new double[0])[getLargestSize(sizeEntry)];
     //double ty = yEntry.getDouble(0); //yEntry.getDoubleArray(new double[0])[getLargestSize(sizeEntry)];
     double tsize = sizeEntry.getDouble(0); //sizeEntry.getDoubleArray(new double[0])[getLargestSize(sizeEntry)];

      // Calculate proportional steering
      double horizontalOffset = tx - 320;
      steerCommand = horizontalOffset * STEER_K;

      // Drive forward until target area is at our desired area
      double targetArea = tsize;
      driveCommand = (DESIRED_TARGET_AREA - targetArea) * DRIVE_K;
      if (driveCommand > MAX_SPEED) { // If max speed is exceeded
          driveCommand = MAX_SPEED; // Set drive command to speed limit
      }
  } else { // If no target is acquired
      driveCommand = 0.0; // Have the robot remain still
      steerCommand = 0.3;
  }
  SmartDashboard.putNumber("driveCommand", driveCommand);
  SmartDashboard.putNumber("steerCommand", steerCommand);
}

  public boolean gripBallFound(NetworkTableEntry sizeEntry){
    double[] sizeArray = sizeEntry.getDoubleArray(new double[0]);
    return sizeArray.length == 0;
  }

  public void setDrive(double steer, double drive, double desiredTargetArea, double maxSpeed) {
    STEER_K = steer;
    DRIVE_K = drive;
    DESIRED_TARGET_AREA = desiredTargetArea;
    MAX_SPEED = maxSpeed;
}

  public static int getLargestSize(NetworkTableEntry sizeEntry) {
    double[] sizeArray = sizeEntry.getDoubleArray(new double[0]);
    int temp = -1; 

    for (int index = 0; index < sizeArray.length; index++) {
      if (sizeArray[index] > sizeArray[temp]) {
        temp = index;
      }
    }
    return temp;
  }
}
