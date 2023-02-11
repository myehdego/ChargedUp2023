// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverStation extends CommandBase {
  /** presuming the robot is at the ramp, 
   * drive up the ramp,
   * go to the middle
   * and than check for balance
   * Based on the balance, go forwards or backwards */
  public SwerveSubsystem driveon;
  private Pose2d endPose;
  private Pose2d fred;
  private PIDController controller;
  private double howfar = RobotConstants.xcg + FieldConstants.chargingstationwidth/2; // Charging station width/2 + xcg
  double encS;
  public DriverStation(SwerveSubsystem driveon) {
    addRequirements(driveon);
    this.driveon = driveon;

    controller = new PIDController(0.3/Math.abs(howfar), 0, 0);  // set robot speed relative to distance from goal
  }

  /* Assuming robot has driven to the edge of the charging station
     and is oriented perpendicular to the edge:
     o set the end pose to be the current pose + the width of the ramp and one apron
     o drive straight ahead in robot frame
     o stop after driving to the end pose
     */
  @Override
  public void initialize() {
    endPose = driveon.getPose();
    // determine starting position
    encS = driveon.returnEncode()[0]; // use one of the four
  }

  // set a drive speed in the robot frame
  @Override
  public void execute() {
    
    double john = controller.calculate(driveon.returnEncode()[0], encS+howfar);
    /* TODO: Ved, there is no law that requires us to use this result linearly.
       Since the controller tends to get too slow near the destination, 
       we could define a function that stays high until it is closer to zero,
       ie, consider y(x) = 1 - x^2 for 0 < x < 1
       */
    System.out.println("Encoder and Target "+driveon.returnEncode()[0] +" " + (encS+howfar));
    System.out.println("Speed "+ john);
    driveon.driveit(john, 0);
    
    SmartDashboard.putNumber("DriveError" , controller.getPositionError());
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
    
   //  double size = pose.getX()*pose.getX() + pose.getY()*pose.getY();
    return Math.abs(controller.getPositionError()) < Math.abs(0.03*howfar);
  }
}
