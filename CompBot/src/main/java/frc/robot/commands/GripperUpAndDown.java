// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GripperUpAndDown extends CommandBase {
  /** makes the Gripper able to go up and down. */
  private Gripper gripper;
  private boolean up;
  public GripperUpAndDown(Gripper gripper, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    addRequirements(gripper);
    this.up = up;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (up) {
      gripper.liftGripper();
    }
    else { 
      gripper.lowerGripper();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
