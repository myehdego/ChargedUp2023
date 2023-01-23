// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverStation extends CommandBase {
  /** Make the robot go up the ramp, go to the middle and balance */
  public SwerveSubsystem driveon;
  private Pose2d endPose;

  
  public DriverStation(SwerveSubsystem driveon) {
    //
    addRequirements(driveon);
    this.driveon = driveon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endPose = driveon.getPose() 
    // TODO next line need to be tranform into robot coordinate system
      .plus(new Transform2d(new Translation2d(FieldConstants.chargingstationlength, 0), new Rotation2d(0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-0.1, 0, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    driveon.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveon.stopModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveon.getPose() .equals(endPose);
  }
}
