// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class FaceGrid extends CommandBase {
  SwerveSubsystem drive;
  PIDController controller;
  /** turn the robot toward the Grid. */
  public FaceGrid(SwerveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    controller = new PIDController(.5/180., 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curAngle = drive.getHeading();
    // since 180  = -180 this could get ugly if we do not ensure monotonicity
    curAngle = curAngle<0?curAngle+360.:curAngle;
    double omega = controller.calculate(curAngle, 180.);
    drive.driveit(0.,0.,omega,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.getPositionError()<3.;
  }
}
