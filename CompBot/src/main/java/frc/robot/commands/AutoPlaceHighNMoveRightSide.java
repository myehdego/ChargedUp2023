// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceHighNMoveRightSide extends SequentialCommandGroup {
  /** Place game piece:
       *    <ol><li> extend arm to desired positiona, but don't wait longer than 3 seconds
       *    <li> open gripper
       *    <li> back away from grid
       *    <li> retract arms
       *  
       *  Step 1 is a race between the ArmRun command and a Wait command. 
       *  Steps 3 and 4 should be accomplished in parallel, but 4 should wait a second before it starts
       */
  public AutoPlaceHighNMoveRightSide(Arm arm, SwerveSubsystem drive, Gripper gripper, PWM lights) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.race(  // first one done ends both
                    new ArmRun(arm, ArmConstants.cubeDepth1,ArmConstants.cubeDepth1R+50, true)  // step 1a
                    ,new WaitCommand(1.5))   // cant wait forever to get in position
                ,new InstantCommand(() -> arm.makeMeDone()),  // ensure step 1a is ended                 
                Commands.race(new ArmRun(arm, ArmConstants.cubeDepth2,ArmConstants.cubeDepth2R, true)  // step 1b
                    ,new WaitCommand(2))   // cant wait forever to get in position                
                ,new GripperOpenClose(gripper, true, lights)  //  step 2 // gripper opens
                ,new WaitCommand(1)
                ,new InstantCommand(() -> arm.makeMeDone())  // ensure step 1b is ended
                ,Commands.parallel(     // drive and retract arm in parallel
                    Commands.race(
                      new DriveGeneric(drive, FieldConstants.leaveCommunityDist+Units.feetToMeters(2),0),   // step 3 //goes back
                      new WaitCommand(4)),
                //new DriveGeneric(drive, Units.feetToMeters(0),0),   // step 3
                    new WaitCommand(1).andThen(new GripperOpenClose(gripper, false, lights)), //  step 2.5 // closes gripper
                    // step 4
                    new WaitCommand(1).andThen(new ArmRun(arm, ArmConstants.retracto0,ArmConstants.retracto0R, true)))  // step 4 // retracts arm
                ,Commands.race( 
                    new DriveGeneric(drive, 0 , Units.feetToMeters(5.5)), // moves to the left
                    new WaitCommand(2))
                ,new DriveGeneric(drive, -FieldConstants.chargingstationwidth/2 , 0) // goes to charging station
      );
  }
}
