// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmRun extends CommandBase {
  Arm arm;
  double Target; 
  boolean Closed = false;
  /** open loop command move arm to a target */
  public ArmRun(Arm arm, double Target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.Target = Target;
  }
  /** Able to change argument between closed and open */
  public ArmRun(Arm arm, double Target, boolean closed) {
    this(arm, Target);
    this.Closed = closed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Closed) {
      // arm.makeMeDone();
      /* try {
        Thread.sleep(0200);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        // e.printStackTrace();
      } */
      arm.pidCoefficient(Math.abs(Target - arm.getExtenderPos()));
      arm.closedLoopController(Target);
    }
    else {
    arm.extend(arm.getExtenderPos() < Target);
    }
    SmartDashboard.putNumber("target", Target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Closed) {
      // arm.closedLoopController(Target);
    }else {}
    SmartDashboard.putBoolean("currentPosTest", arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance 
    && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtend();
    SmartDashboard.putString("target", "quit");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance 
    && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance
    || arm.amIDone();
  }
}
