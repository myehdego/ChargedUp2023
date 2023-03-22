// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GamePieceCam;
import frc.robot.subsystems.SwerveSubsystem;

public class twist extends CommandBase {
  /** Creates a new twist. */
  double twistdegrees;
  PIDController controller;
  boolean stopwhendone;
  double endangle, currentangle;
  SwerveSubsystem drive;
  Timer timer;
  GamePieceCam gamePieceCam;
  public twist(SwerveSubsystem drive, GamePieceCam gamePieceCam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    // this.twistdegrees = twistdegrees;
    this.gamePieceCam = gamePieceCam;
    controller = new PIDController(0,0 ,0 );
    timer=new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // double endangle = drive.getPose().getRotation().getDegrees()+twistdegrees;
    controller.setP(1./22.);
    System.out.println("twist started");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = gamePieceCam.getYaw();
    currentangle = drive.getPose().getRotation().getDegrees();
    double resolve;
    if(yaw > 40.) {
      resolve = 0.3;
    }
    else {
      resolve = controller.calculate(yaw, 0.);
    }
    //double resolve = controller.calculate(currentangle, endangle);
    double speed;
    // if(twistdegrees > 0) {
    //   resolve = .3;
    // }
    // else {
    //   resolve = -.3;
    // }
    drive.driveit(0, 0, resolve, stopwhendone);
    System.out.println("resolve "+ resolve);
    //System.out.println(currentangle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopModules();
    System.out.println(currentangle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(currentangle - endangle) < 3 || Math.abs(currentangle+360-endangle)<3||
    return timer.hasElapsed(2.5) || controller.getPositionError() < 1;
  }
}
