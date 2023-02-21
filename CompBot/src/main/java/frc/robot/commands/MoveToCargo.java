// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagCamera;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToCargo extends CommandBase {
  /** Moves to the cargo position on the field */
  AprilTagCamera camera;
  SwerveSubsystem swerveSubsystem;
  public MoveToCargo(AprilTagCamera camera, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.camera = camera;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(camera);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  camera.getDistanceToCargo();  //TODO: move robot to cargo location
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
