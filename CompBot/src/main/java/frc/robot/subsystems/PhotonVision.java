// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CamConstant;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  /** Using a camera to get data from AprilTags
   * The data recived would be the area, pitch and yaw of an April tag 
   * Using the data we recive from the april tags we can determine the position of our robot relative to the tag
  */
  public PhotonVision() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }
  
  private PhotonTrackedTarget target;
  /** look for info on any recognized april tag:
   *   publish result to Network Table
   */
  public int getAprTag() {
    if(result != null) {
      target = result.getBestTarget();
      int id = target.getFiducialId();
      SmartDashboard.putNumber("aprTag#",id);
      return id;
    }
    return 999;
  }

  /** determine the location on the field of the robot
   * from its orientation relative to a visible April Tag
   * 
   * returns a Pose2d 
   */
  public Pose2d getRobotPosition() {
    Transform3d campos = target.getBestCameraToTarget();
    int targetid = getAprTag();
    /*
     * Assume for now the Pitch angle of the camera is zero
    double lxyz = Math.pow(campos.getX(),2.) + 
     Math.pow(campos.getY(),2.) +  
     Math.pow(campos.getZ(),2.);
    double lxy = lxyz * Math.cos(CamConstant.PitchAngle);  // TODO missing a piece
    */
    double x = campos.getX();   // TODO finish this
    double y = campos.getY();
    double ang = campos.getRotation().getY();


      return new Pose2d(x ,y, new Rotation2d(ang));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /* Look for an april tag */
    result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("hasTarget", hasTargets);
    
  }

  private static final double taglocations[] [] = {
    {610.77, 42.19, 18.22},
    {610.77, 108.19, 18.22},
    {610.77, 174.19, 18.22},
    {636.96, 265.74, 27.38},
    { 14.25, 265.74, 27.38}, 
    { 40.45, 174.19, 18.22},
    { 40.45, 108.19, 18.22},
    { 40.45, 42.19, 18.22}
    
  };
}
