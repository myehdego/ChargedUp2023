// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Face extends CommandBase {
  SwerveSubsystem drive;
  PIDController controller;
  double heading;
  /** turn the robot toward the Grid. */
  public Face(SwerveSubsystem drive, double heading) {
    this.drive = drive;
    this.heading = heading;
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
    double omega = controller.calculate(curAngle, heading);
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
