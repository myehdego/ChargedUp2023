// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceCam extends SubsystemBase {
  private AnalogInput pixyCam;
  private AnalogInput pixyCamPin1;
  //private static final int center = 1600;  // analog x
  //private static final int maxval = 3200;  // analog x
  private static final int center = 1000;  // analog y  TODO need to calibrate these
  private static final int maxval = 2000;  // analog y
  private static final double fov = 45; //degrees
  private boolean iSeeSomething = false;
  private static final int gotOneThreshold = 3000;  // TODO need to calibrate
  /** Use Pixy Camera to recognize a game piece
   *   and return info about it. 
   *  Current implementation returns the yaw angle
   *   from directly forward in degrees*/
  public GamePieceCam() {
    pixyCam = new AnalogInput(0);  //TODO probably ought to define these connections in Constants.java
    pixyCamPin1 = new AnalogInput(1);
  }

  /** return the yaw angle to the observed game piece
   *  relative to directly forward, degrees [-22, 22]
   */
  public double getYaw() {
    double val = (pixyCam.getAverageValue()-center)*(fov/maxval);
    //return val;  // when installed normally or on its right side
    return -val;  // when installed on its left side
  }

  public boolean isVisible() {
    return iSeeSomething;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    iSeeSomething = pixyCamPin1.getAverageValue() > gotOneThreshold;
  }
}
