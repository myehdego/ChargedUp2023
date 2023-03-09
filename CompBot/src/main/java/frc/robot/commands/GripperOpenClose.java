// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.Lights;
import frc.robot.subsystems.Gripper;

public class GripperOpenClose extends CommandBase {
  /** Open and closes the gripper. */
  private Gripper gripper;
  private boolean open;
  private Timer timer;
  private double waitTime;
  PWM lights;
  //private boolean expel;
  public GripperOpenClose(Gripper gripper, boolean open, PWM lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    addRequirements(gripper);
    this.open = open;
    timer = new Timer();
    this.lights = lights;
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
        lights.setSpeed(Lights.RED);
      }
      else {
        gripper.rollerSpit();
        if(gripper.getPieceType() == GripperConstants.CUBE) lights.setSpeed(Lights.CUBE_COLOR);
        else lights.setSpeed(Lights.CONE_COLOR);
      }
    }
    else {
      waitTime=0.4;
      gripper.closegripper();
      //gripper.rollersStop();
      lights.setSpeed(Lights.GREEN);
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
