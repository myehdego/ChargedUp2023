// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverStation extends CommandBase {
  /** Make the robot go up the ramp, go to the middle and balance */
  public SwerveSubsystem driveon;
  private Pose2d endPose;
  private Pose2d fred;
  private PIDController controller;
  public DriverStation(SwerveSubsystem driveon) {
    //
    addRequirements(driveon);
    this.driveon = driveon;

    controller = new PIDController(0.1/1.5, 0, 0);
  }

  /* Assuming robot has driven to the edge of the charging station
     and is oriented perpendicular to the edge:
     o set the end pose to be the current pose + the width of the ramp and one apron
     o drive straight ahead in robot frame
     o stop after driving to the end pose
     */
  @Override
  public void initialize() {
    endPose = driveon.getPose() 
    // TODO next line need to be tranform into robot coordinate system
     // .plus(new Transform2d(new Translation2d(FieldConstants.chargingstationlength, 0),
      .plus(new Transform2d(new Translation2d(-1.55, 0),
            new Rotation2d(0)));
            fred = driveon.getPose().relativeTo(endPose);
  }

  // set a drive speed in the robot frame
  @Override
  public void execute() {
  


   /*  ChassisSpeeds chassisSpeeds = 
          new ChassisSpeeds(-0.1, 0, 0);
    SwerveModuleState[] moduleStates = 
          DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          // TODO make this work for the comp bot, too
    driveon.setModuleStates(moduleStates); */
  }

  // stop the robot.
  @Override
  public void end(boolean interrupted) {
    driveon.stopModules();
  }

  // Returns true when the robot has reached the desired end pose.
  @Override
  public boolean isFinished() {
   // SmartDashboard.putNumber(endPose, 0);
    Pose2d pose = driveon.getPose().relativeTo(endPose);
   //  double size = pose.getX()*pose.getX() + pose.getY()*pose.getY(); 
    return (pose.getX()*fred.getX() < 0.) && (pose.getY()*fred.getY() < .0);
  }
}
