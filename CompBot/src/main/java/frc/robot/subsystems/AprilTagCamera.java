// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CamConstant;

public class AprilTagCamera extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  /** Using a camera to get data from AprilTags
   * The data recived would be the area, pitch and yaw of an April tag 
   * Using the data we recive from the april tags we can determine the position of our robot relative to the tag
  */
  public AprilTagCamera() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }
  
  private PhotonTrackedTarget target;
  /** look for info on any recognized april tag:
   *   publish result to Network Table
   */
  public int getAprTag() {
    if(result != null) {
      target = result.getBestTarget();
      if(target == null){
      return 998;
      }
      int id = target.getFiducialId();
      //SmartDashboard.putNumber("aprTag#",id);
      return id;
    }
    return 999;
  }

  /** determines the location on the field of the robot
   * from its orientation relative to a visible April Tag.
   * 
   * <p> returns a Pose2d, dimensions in meters relative to corner of blue grid
   */
  public Pose2d getRobotPosition() {
    try {
      Transform3d campos = target.getBestCameraToTarget();  // dimensions in meters
      int targetid = getAprTag();
      if (targetid<1)throw new NullPointerException("Tag id not found");
      /*
      * Assume for now the Pitch angle of the camera is zero
      */
      /* 0.Presuming we are facing the tag head on
      * 1. Get a April tag id
      * 2. Get the x and y of the apriltag
      * 3. Get the x and y offset of the camera relative to the tag
      * 4. the camera position is  the april tag + the offset 
      * 5. The robot position is the camera position + the offset of the camera relative to the robot
      */
      double camx = campos.getX();
      double camy = campos.getY();
      //double ang = campos.getRotation().getY();
      double tagx = Units.inchesToMeters(taglocations[targetid-1][0]);
      double tagy = Units.inchesToMeters(taglocations[targetid-1][1]);
      double camlocationx = tagx + ((targetid>4)?camx:(-camx));
      double camlocationy = tagy + ((targetid>4)?camy:(-camy));
      double robotx = camlocationx + CamConstant.CameraLocationX;
      double roboty = camlocationy + CamConstant.CameraLocationY;
      //SmartDashboard.putNumber("TagX", tagx);
      //SmartDashboard.putNumber("TagY", tagy);
      //SmartDashboard.putNumber("Camx", camx);
      //SmartDashboard.putNumber("CamY", camy);
     
      return new Pose2d(Units.metersToInches(robotx),
                        Units.metersToInches(roboty), new Rotation2d(0.)); // TODO: Presumes we are facing the tag head on
    }catch(NullPointerException e){
      return new Pose2d(-999.,-999., new Rotation2d(-999.));
    }
  }

  public double getDistanceToCargo() {
    //Calculates the distance between the robot and the bottom left target on the field
    Pose2d cargolocation = new Pose2d(264.45,36.19,new Rotation2d(0.));
    double distancetoCargo = PhotonUtils.getDistanceToPose(getRobotPosition(),cargolocation);
    //SmartDashboard.putNumber("distancetocargo", distancetoCargo);
    return distancetoCargo;
  }
  
  public boolean seesTarget() {
    return camera.getLatestResult().hasTargets();
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
    {610.77, 42.19, 18.22},   // data relative to the right corner
    {610.77, 108.19, 18.22},  // of the blue grid,
    {610.77, 174.19, 18.22},  // x pointing from blue toward red
    {636.96, 265.74, 27.38},  // y pointing toward red loading station,
    { 14.25, 265.74, 27.38},  // inches
    { 40.45, 174.19, 18.22},
    { 40.45, 108.19, 18.22},
    { 40.45, 42.19, 18.22}
  };
  private static final double  cargolocations [] [] = {
    {224.00,48.,0.}
  };
  
}
