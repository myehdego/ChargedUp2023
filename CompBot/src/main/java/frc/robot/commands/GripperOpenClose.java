// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GripperOpenClose extends CommandBase {
  /** Open and closes the gripper. */
  private Gripper gripper;
  private boolean open;
  private Timer timer;
  private double waitTime;
  //private boolean expel;
  public GripperOpenClose(Gripper gripper, boolean open) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    addRequirements(gripper);
    this.open = open;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    if (open) {
      waitTime=0;
      gripper.opengripper();
      if (gripper.expel()) 
      {
        gripper.rollersGo();
      }
      else {
        gripper.rollerSpit();
      }
    }
    else {
      waitTime=1.;
      gripper.closegripper();
      //gripper.rollersStop();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!open)gripper.rollersStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(waitTime)){
      timer.stop();
      timer.reset();
        return true;
    }else {
      //timer.stop();
      return false;
    }
  }
}
