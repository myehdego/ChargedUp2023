// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class PreRetract extends CommandBase {
  Arm arm;
  double Target;
  double RaiserTarget;
  boolean Closed = false;
  /** Move the arm much like ArmRun
   * but with low finishing tolerance
   * to aid in avoiding calamity when retracting */
  public PreRetract(Arm arm, double Target, double RaiserTarget, boolean closed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.Target = Target;
    this.RaiserTarget = RaiserTarget;
    this.Closed = closed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Closed) {
      arm.pidCoefficient(Math.abs(Target - arm.getExtenderPos()), Math.abs(RaiserTarget - arm.getRaiserPos()));
      arm.closedLoopController(Target, RaiserTarget);
    } else {
      arm.extend(arm.getExtenderPos() < Target);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // probably no need to stop motors if ArmRun
    // immediately succeeds this command
    arm.stopExtend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance*3
        && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance*3
        // TODO: is check on raiser position required?
        || arm.amIDone();
  }
}
