// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CamConstant;

public class PhotonVision extends SubsystemBase {
  PhotonCamera camera;
  PhotonPipelineResult result;
  /** Using a camera to get data from AprilTags
   * The data recived would be the area, pitch and yaw of an April tag 
   * Using the data we recive from the april tags we can determine the position of our robot relative to the tag
  */
  public PhotonVision() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    
  }
  
  PhotonTrackedTarget target;
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

  public Transform2d getTargetPosition() {
    Transform3d campos = target.getBestCameraToTarget();
    double lxyz = Math.pow(campos.getX(),2.) + 
     Math.pow(campos.getY(),2.) +  
     Math.pow(campos.getZ(),2.);
    double lxy = lxyz * Math.cos(CamConstant.PitchAngle);  // TODO missing a piece

    double x = campos.getX() * lxy;   // TODO finish this
    double y = campos.getY();
    double ang = campos.getRotation().getY();

      return new Transform2d(new Translation2d(x ,y), new Rotation2d(ang));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /* Look for an april tag */
    result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("hasTarget", hasTargets);
    
  }


}
