// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagCamera;

public class GetAprilTag extends CommandBase {
  AprilTagCamera camera;
  /**Uses the PhotonVision subsystem to detect AprilTags and display the tag id */
  public GetAprilTag(AprilTagCamera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera);
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.getAprTag();  // TODO  who cares?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
