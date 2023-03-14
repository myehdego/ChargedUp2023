// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class StoponDockGeneric extends SequentialCommandGroup {
  /** drive from grid, past the Charging Station
   *  drive sideways infront of the Charging station
   *  drive back on charging station 
   *  balance
   */
  public StoponDockGeneric(SwerveSubsystem drive, double sideDist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveGeneric(drive, FieldConstants.chargingstationwidth, 0, true),
      new DriveGeneric(drive, 0, sideDist, true),
      new Docked(drive), 
      new DriveGeneric(drive, FieldConstants.chargingstationwidth/2, 0, true)
    );
  }
}
