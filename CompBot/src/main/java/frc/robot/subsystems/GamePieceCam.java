// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceCam extends SubsystemBase {
  private AnalogInput pixyCam;
  private static final int center = 1600;
  private static final int maxval = 3200;
  private static final double fov = 45; //degrees
  /** Use Pixy Camera to recognize a game piece
   *   and return info about it. 
   *  Current implementation returns the yaw angle
   *   from directly forward in degrees*/
  public GamePieceCam() {
    pixyCam = new AnalogInput(0);
  }

  /** return the yaw angle to the observed game piece
   *  relative to directly forward, degrees [-22, 22]
   */
  public double getYaw() {
    return (pixyCam.getAverageValue()-center)*(fov/maxval);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
