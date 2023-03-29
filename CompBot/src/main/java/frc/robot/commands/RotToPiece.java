// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.GamePieceCam;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotToPiece extends PIDCommand {
  SwerveSubsystem drive ;
  GamePieceCam camera;
  /** rotate the robot to face a game piece
   */
  public RotToPiece(SwerveSubsystem drive, GamePieceCam camera) {
    super(
        // The controller that the command will use
        new PIDController(.4/22., 0, 0),    // P = .4 * 1./(FOV/2)
        // This should return the measurement
        () -> {
                //return camera.getYaw()>40? 22.: camera.getYaw();
                return camera.isVisible()? camera.getYaw(): 22.;
              },
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          //drive.driveMe(output);
          drive.driveit(0.,0.,output,true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drive=drive;
    this.camera = camera;
    addRequirements(drive, camera);
    // Configure additional PID options by calling `getController` here.
  }

  /* @Override
  public void execute() {
    super.execute();
    System.out.println(getController().getPositionError());
  } */

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(camera.getYaw())<5.;
  }
}
