// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class NudgeDrive extends CommandBase {
  /** nudges the drive system 3in
   *  to the left or right in robot coord system. */
  private SwerveSubsystem nudger;
  private int forward, right;
  private double encS, target;
  private PIDController controller;
  public NudgeDrive(SwerveSubsystem nudger, int forward, int right) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nudger);
    this.nudger = nudger;
    this.forward = forward;
    this.right = right;
    controller = new PIDController(0.3/Math.abs(Units.inchesToMeters(3.)), 0, 0);  // set robot speed relative to distance from goal
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double [] postition = nudger.returnEncode();
    encS = 0.;
    for (double each : postition) {
      encS = encS + each * .25;
    }
    target = encS + Units.inchesToMeters(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double [] postition = nudger.returnEncode();
    double whereiam = 0.;
    for (double each : postition) {
      whereiam = whereiam + each * .25;
    }
    double speed = controller.calculate(whereiam, target);
    nudger.driveit(0,speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //nudger.setbrakemode();
    nudger.stopModules();
    //nudger.setcoastmode();  // too quick succession to matter
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("nudger error", Math.abs(controller.getPositionError())); 
    SmartDashboard.putNumber("nudger tol", Math.abs(Units.inchesToMeters(0.3)));
    return Math.abs(controller.getPositionError()) < Math.abs(Units.inchesToMeters(0.3));
  }
}
