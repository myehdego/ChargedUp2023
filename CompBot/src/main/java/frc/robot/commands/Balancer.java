// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.WPI_Pigeon2;


public class Balancer extends CommandBase {
  private SwerveSubsystem drive;
  private PIDController controller;
  //private WPI_Pigeon2 gyro;
  private WPI_Pigeon2 pigeon;
  double zero;
  private PWM lights;
/** drives to balance on the station based on pitch angle. */
  public Balancer(SwerveSubsystem drive, WPI_Pigeon2 gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive=drive;
    this.pigeon = gyro;
    controller = new PIDController(AutoConstants.kPBalancer, 0, 0);
  }

  /* steps to make it happen:
   *   define PIDController parameters
   *     how far can angle be away from target?
   *     how fast should robot move?
   *   target is "zero"
   *   target must be maintained, not just reached momentarily
   *      => define "finished" criterion appropriately
   *   use PIDcontroller to get drive response
   *   run the drive
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //y is to the left so positive pitch is nose down
    //zero = pigeon.getPitch();   // who cares?
    //controller.setP(AutoConstants.kPBalancer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   zero =  Math.sin(Math.toRadians(pigeon.getPitch()-1));
   double balancer = controller.calculate(zero, 0);
   drive.driveit(-balancer, 0, 0, false);
   /* if need pitch rate, see pigeon.getRawGyro */
   // do we need to ensure robot stays aligned with field?
  };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // run balancer until the end of the match - never end
  }
}
