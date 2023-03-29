// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class StoponDockMiddle extends SequentialCommandGroup {
  /** drive from grid, over the Charging Station
   *  drive back on charging station 
   *  balance
   */
  public StoponDockMiddle(SwerveSubsystem drive, WPI_Pigeon2 gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.race(
        new DriveGeneric(drive, FieldConstants.chargingstationwidth+FieldConstants.Gridtostation+Units.feetToMeters(2.5), 0, true),
        new WaitCommand(4)
      ),
      new Docked(drive),
      Commands.race(
        //new DriveGeneric(drive, -FieldConstants.chargingstationwidth/2-Units.feetToMeters(2.5), 0, true),
        new DriveGenericHead(drive, -FieldConstants.chargingstationwidth/2-Units.feetToMeters(2.5), 0, 180.),
        new WaitCommand(1)
      ),
      new Balancer(drive, gyro)
    );
  }
}
