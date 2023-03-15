// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmRun extends CommandBase {
  Arm arm;
  double Target;
  double RaiserTarget;
  boolean Closed = false;
  boolean floor = false;

  /** open loop command move arm to a target */
  public ArmRun(Arm arm, double Target, double RaiserTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.Target = Target;
    this.RaiserTarget = RaiserTarget;
    //System.out.println("RaiserTarget " + RaiserTarget);
  }

  /** command move arm to a target */
  public ArmRun(Arm arm, double Target, double RaiserTarget, boolean closed) {
    this(arm, Target, RaiserTarget);
    this.Closed = closed;
    this.floor = false;
  }
  public ArmRun(Arm arm, double Target, double RaiserTarget, boolean closed, boolean floor) {
    this(arm, Target, RaiserTarget, closed);
    this.floor = floor;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Closed) {
      arm.pidCoefficient(Math.abs(Target - arm.getExtenderPos()), Math.abs(RaiserTarget - arm.getRaiserPos()));
      arm.closedLoopController(Target, RaiserTarget, floor);
    } else {
      arm.extend(arm.getExtenderPos() < Target);
      arm.raise(arm.getRaiserPos() < RaiserTarget);
    }
    //SmartDashboard.putNumber("target", Target);
    //SmartDashboard.putNumber("raiserTarget", RaiserTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Closed) {
      // arm.closedLoopController(Target);
    } else {
      if (arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance
       && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance) arm.stopExtend();
      if (arm.getRaiserPos() >= Target - ArmConstants.raiserTolerance
       && arm.getRaiserPos() <= Target + ArmConstants.raiserTolerance) arm.stopRaise();
    }
    //SmartDashboard.putBoolean("currentPosTest", arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance
      //  && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance);
    //SmartDashboard.putString("RiserIssue", "target, pos: " + RaiserTarget + " " + arm.getRaiserPos());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtend();
    arm.stopRaise();
    //SmartDashboard.putString("target", "quit");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance
        && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance
        // TODO: OK for closed loop, but bad for open loop
        && arm.getRaiserPos() >= Target - ArmConstants.raiserTolerance
        && arm.getRaiserPos() <= Target + ArmConstants.raiserTolerance
        || arm.amIDone();
  }
}
