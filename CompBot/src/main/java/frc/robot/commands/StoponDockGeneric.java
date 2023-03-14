// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class StoponDockGeneric extends SequentialCommandGroup {
  /** drive from grid, past the Charging Station
   *  drive sideways in front of the Charging station
   *  drive back on charging station 
   *  balance
   */
  public StoponDockGeneric(SwerveSubsystem drive, WPI_Pigeon2 gyro, double sideDist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.race(
           new DriveGeneric(drive, FieldConstants.leaveCommunityDist+Units.feetToMeters(2),0),   // step 1
           new WaitCommand(4)),
      Commands.race( 
            new DriveGeneric(drive, 0 , sideDist), // step 2
            new WaitCommand(2)),
      new Docked(drive), 
      new DriveGeneric(drive, FieldConstants.chargingstationwidth/2, 0, true)
      ,new Balancer(drive, gyro)
    );
  }
}
