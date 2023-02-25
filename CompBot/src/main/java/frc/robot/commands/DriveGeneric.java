// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveGeneric extends CommandBase {
  /** Drive a given distance in any direction
   * In field coordinates
   */
  int randomNumber = Math.floorMod(System.currentTimeMillis(), 1000);
  SwerveSubsystem driver;
  Pose2d startpose, targetpose;
  double xdist;
  double ydist;
  double encS;
  PIDController controller;
  double target, dist;
  double tol;
  boolean stopwhendone = true;
  boolean iShouldStop = false;
  public DriveGeneric(SwerveSubsystem driveon, double xdist, double ydist, boolean stopwhendone) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveon);
    this.driver = driveon;
    this.xdist = xdist;
    this.ydist = ydist;
    this.stopwhendone = stopwhendone;
    controller = new PIDController(0, 0, 0);  // set p in init
  }

  public DriveGeneric(SwerveSubsystem driveon, double xdist, double ydist) {
    this(driveon, xdist, ydist, true);
  }

  /** stop DriveGeneric Command whether accomplished or not */
  public void endDriveGeneric() {
    iShouldStop = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driver.makemefalse();
    // encS = driver.returnEncode()[2];
    dist = Math.sqrt(xdist*xdist+ydist*ydist);
    // target = encS+dist;
    tol = 0.02*dist;
    controller.setP(.3/dist);
    startpose = driver.getPose();
    Transform2d transform = new Transform2d(new Translation2d(xdist, ydist), new Rotation2d(0));
    targetpose = startpose.plus(new Transform2d(new Translation2d(xdist, ydist), new Rotation2d(0)));
    SmartDashboard.putString("Transform", transform.toString());
    SmartDashboard.putString("TargetPose", targetpose.toString());
    SmartDashboard.putString("StartPose", startpose.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentpose = driver.getPose();
    double whereiam = PhotonUtils.getDistanceToPose(currentpose, startpose);
    double speed = controller.calculate(whereiam, dist);
    driver.driveit(speed*xdist/dist, speed*ydist/dist, 0, true);

    SmartDashboard.putString("Error", controller.getPositionError() + " < " + tol);
    SmartDashboard.putString("CurrentPose", currentpose.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopwhendone)
      driver.stopModules();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.getPositionError() < tol ||
    //iShouldStop;
    driver.shouldistop();
  }
}
