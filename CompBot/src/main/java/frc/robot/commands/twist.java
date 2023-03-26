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
  double endangle, currentangle;
  PIDController controller;
  SwerveSubsystem drive;
  Timer timer;  // give up after a short while
  GamePieceCam gamePieceCam;
  private boolean closedLoop=true;
  public twist(SwerveSubsystem drive, double twistdegrees) {
    addRequirements(drive);
    this.drive = drive;
    this.twistdegrees = twistdegrees;
    controller = new PIDController(.4/Math.abs(twistdegrees),0 ,0 );
    timer=new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //controller.setP(1./22.);
    endangle = drive.getPose().getRotation().getDegrees()+twistdegrees;
    // TODO: is endangle in the range of getDegrees()? 
    //System.out.println("twist started");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentangle = drive.getPose().getRotation().getDegrees();
    double resolve;
    if (closedLoop){
      resolve = controller.calculate(currentangle, endangle);
    } else {
      resolve = Math.copySign(.3, endangle-currentangle);
    }
    
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
    boolean done;
    if (closedLoop) done=controller.getPositionError() < 1.;
    else done = Math.abs(endangle - currentangle) < 2.;
    return timer.hasElapsed(2.5)   // give up after a short while
          || done;  // or quit when accomplished
  }
}
