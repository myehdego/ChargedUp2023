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
  /** Creates a new ArmRun. */
  public ArmRun(Arm arm, double Target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.Target = Target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.extend(arm.getExtenderPos() < Target);
    SmartDashboard.putNumber("target", Target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("currentPosTest", arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance 
    && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtend();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getExtenderPos() >= Target - ArmConstants.retractorTolerance 
    && arm.getExtenderPos() <= Target + ArmConstants.retractorTolerance;
  }
}
