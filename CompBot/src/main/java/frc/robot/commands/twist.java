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
  /** Rotate robot to face a game piece */
  double twistdegrees;
  double endangle;
  PIDController controller;
  SwerveSubsystem drive;
  Timer timer;  // give up after a short while
  GamePieceCam gamePieceCam;
  public twist(SwerveSubsystem drive, double twistdegrees) {
    addRequirements(drive);
    this.drive = drive;
    this.twistdegrees = twistdegrees;
    controller = new PIDController(.4/Math.abs(twistdegrees),0 ,0 );
    timer=new Timer();
    // TODO: should it stop rotating if it has gone around once and not found a game piece?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //controller.setP(1./22.);
    endangle = drive.getPose().getRotation().getDegrees()+twistdegrees;
    System.out.println("twist started");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentangle = drive.getPose().getRotation().getDegrees();
    double resolve;
    //if(gamePieceCam.isVisible()) {
      //resolve = controller.calculate(gamePieceCam.getYaw(), 0.);
      resolve = controller.calculate(currentangle, endangle);
    
    //resolve = Math.min(resolve,.7);
    drive.driveit(0, 0, resolve, false);
    // System.out.println("resolve "+ resolve);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopModules();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2.5)   // give up after a short while
          || controller.getPositionError() < 1.;  // or quit when accomplished
  }
}
