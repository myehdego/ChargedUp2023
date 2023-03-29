// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax retractorMotor;
  CANSparkMax retractorMotorfollower;
  CANSparkMax raiserMotor;
  CANSparkMax raiserMotorfollower;
  private SparkMaxPIDController retractorPidController;
  private SparkMaxPIDController raiserPidController;
  private RelativeEncoder retractorEncoder;
  private RelativeEncoder retractorfollowerEncoder;
  private RelativeEncoder raiserEncoder;
  double kp;
  double rkp;
  private boolean IamDone;  // mechanism to interrupt a Command using the arm
  private boolean closed = false;
  private boolean closedR = false;
  // TODO IamDone interrupts the retractor motor controller; need a separate one for raiser?
  /** The arm will be able to have a range of motion cosisting of going up, down, extend, 
   * and retract to have the ability to reach futher up on the shelves and pegs  */
  public Arm() {
    retractorMotor = new CANSparkMax(CANIDs.ArmRetractorMotor, MotorType.kBrushless);
    retractorMotor.restoreFactoryDefaults();
    retractorMotor.setInverted(CANIDs.retractorMotorInverted);
    retractorMotor.setIdleMode(IdleMode.kBrake);

    retractorMotorfollower = new CANSparkMax(CANIDs.ArmRetractorMotorfollower, MotorType.kBrushless);
    retractorMotorfollower.restoreFactoryDefaults();
    retractorMotorfollower.follow(retractorMotor,true);
    retractorMotorfollower.setIdleMode(IdleMode.kBrake);

    raiserMotor = new CANSparkMax(CANIDs.ArmRaiserMotor, MotorType.kBrushless);
    raiserMotor.restoreFactoryDefaults();
    raiserMotor.setInverted(CANIDs.ArmRaiserMotorInverted);
    raiserMotor.setIdleMode(IdleMode.kBrake);

    raiserMotorfollower = new CANSparkMax(CANIDs.ArmRaiserMotorfollower, MotorType.kBrushless);
    raiserMotorfollower.restoreFactoryDefaults();
    raiserMotorfollower.follow(raiserMotor,true);
    raiserMotorfollower.setIdleMode(IdleMode.kBrake);

    retractorEncoder = retractorMotor.getEncoder();
    retractorfollowerEncoder = retractorMotorfollower.getEncoder();
    retractorPidController = retractorMotor.getPIDController();
    raiserPidController = raiserMotor.getPIDController();
    retractorEncoder.setPositionConversionFactor(ArmConstants.retractorEncoderScale);  //  degrees
    retractorfollowerEncoder.setPositionConversionFactor(ArmConstants.retractorEncoderScale);  //  degrees
    raiserEncoder = raiserMotor.getEncoder(); 
    raiserEncoder.setPositionConversionFactor(ArmConstants.raiserEncoderScale);  //  degrees

    retractorMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.retractorForwardLimit);
    retractorMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.retractorReverseLimit);
    retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    retractorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    raiserMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.raiserForwardLimit);
    raiserMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.raiserReverseLimit);
    raiserMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    raiserMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  /** disable soft limit for the reverse direction of the retractor motor
   * ( for use during set up -- during test mode )
   */
  public boolean softLimitONOFF() {
    if (retractorMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse)) {
      retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      retractorMotor.enableSoftLimit(SoftLimitDirection.kForward, false); 
      raiserMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      raiserMotor.enableSoftLimit(SoftLimitDirection.kForward, false); 
    }
    else { 
      retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true); 
     retractorMotor.enableSoftLimit(SoftLimitDirection.kForward, true); 
     raiserMotor.enableSoftLimit(SoftLimitDirection.kReverse, true); 
     raiserMotor.enableSoftLimit(SoftLimitDirection.kForward, true); 
    }
    return retractorMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    
  }
  
  /** raise forarm */
  public void raise() {
    raiserMotor.set(ArmConstants.TEST_SPEED);
  }
  /** raise forarm on true,
   *  lower on false
  */
  public void raise(boolean direction) {
    if(direction) raise();
    else lower();
  }
  /** raise forarm at desired power 
   *  at a specified power - positive extends, negative retracts
   */
  public void raise(double speed) {
    raiserMotor.set(speed);
  }

  /** lower forarm */
  public void lower() {
    raiserMotor.set(-0.5);
  } 

  public void lower(double speed){
    raiserMotor.set(-speed);
  }

  /** report status of Arm Commands */
  public boolean amIDone() {
    return IamDone;
  }

  /** enforce completion of Arm Commands */
  public void makeMeDone() {
    IamDone = true;
    closed = false;
    closedR = false;
  }

  public void makeMeUndone() {
    IamDone = false;
  }

  /** update P parameter for retractor motor closed loop controller. */
  public void pidCoefficient(double distance, double raiserdistance) {
    kp = 1 * ArmConstants.RetractorP / distance;
    rkp = 1 * ArmConstants.RaiserP / distance;
    retractorPidController.setP(kp);
    raiserPidController.setP(rkp);
  }

  double latestTargetE;  // target position for retractor
  double latestTargetR;  // target position for raiser
  /** run retractor motor closed loop controller */
  public void closedLoopController(double Target, double RaiserTarget, boolean floor) {
    IamDone = false;
    closed = true;
    closedR = true;
    latestTargetE = Target;
    latestTargetR = RaiserTarget;
    retractorPidController.setReference(Target, ControlType.kPosition);
    raiserPidController.setReference(RaiserTarget+(floor?temp_raiserIncrement:0.), ControlType.kPosition);
  }
  public void closedLoopController(double Target, double RaiserTarget) {
    closedLoopController(Target, RaiserTarget,false);
  }
  
  double temp_raiserIncrement=0.;
  /**increment the position from the floor for the selected game piece.
   * 
   * @param cube true, else set for cone
   */
  public void incForCube (boolean cube) {
    if(cube)temp_raiserIncrement = 80;
    else temp_raiserIncrement = 0.;
  }

  /**  Adjusts the target for the retractor pidcontroller
   *     or runs the retractor.
   *   off stops it
   */
  public void retargetRetract(double Adjustment, boolean off) {
    if (closed) {
      latestTargetE += Adjustment;
      retractorPidController.setReference(latestTargetE, ControlType.kPosition);
    } else {
      if (off){
         extend(0.);
      }else {
        //extend(Adjustment>0.);
      }
    }
  }
  public void retargetRetract(double Adjustment) {
    retargetRetract(Adjustment,false);
  }

  /** Adjust the target for the raiser pidcontroller
   *   or
   * runs the raiser when in open loop-mode.
   *   off stops it
   */
  public void retargetRaise(double Adjustment, boolean off) {
    if (closedR) {
      latestTargetR += Adjustment;
      raiserPidController.setReference(latestTargetR, ControlType.kPosition);
    } else {
      if (off){
        raise(0.);
        System.out.println("de nudge");
      }else {
        //raise(Adjustment>0.);
        System.out.println("nudge");
      }
    }
  }
  /** Adjust the target for the raiser pidcontroller */
  public void retargetRaise(double Adjustment) {
    retargetRaise(Adjustment,false);
  }
  
  boolean Floor;   //true if its on the floor
  /** sets whether the arm is in floor position */
  public void setFloor(boolean Floor) 
  {
    this.Floor = Floor;
  }

  public boolean getFloor() {
    return Floor;
  }

  /** extend the upper arm  */
  public void extend() {
    retractorMotor.set(ArmConstants.TEST_SPEED);
  }
  /** Extends if boolean "direction" is true, retracts if false */
  public void extend(boolean direction) {
    if(direction) extend();
    else retract();
  }
  /** Extend the arm
   *  at a specified power - positive extends, negative retracts
   */
  public void extend(double speed){
    retractorMotor.set(speed);
  }

  /** retract the upper arm */
  public void retract() {
    retractorMotor.set(-0.5);
  }
  public void retract(double speed) {
    retractorMotor.set(-speed);
  }

  public void stopRaise() {
    raiserMotor.stopMotor();
  }
  public void stopExtend() {
    retractorMotor.stopMotor();
  }

  public double getRaiserPos() {
    return raiserEncoder.getPosition();
  }
  public double getExtenderPos() {
    return retractorEncoder.getPosition();
  }
  public double getExtenderfPos() {
    return retractorfollowerEncoder.getPosition();
  }

  public void resetEncoders() {
    retractorEncoder.setPosition(0);
    retractorfollowerEncoder.setPosition(0);
    raiserEncoder.setPosition(0);
  }

  /** This runs open-loop controller, to extend or retract towards target */
  public CommandBase extensionCommand(double Target) {
    return runOnce(() -> {
              extend(retractorEncoder.getPosition() < Target);
              //SmartDashboard.putNumber("target", Target);
        }) 
      .andThen(() -> {
        //extend();
              /* SmartDashboard.putBoolean("currentPosTest", 
              retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance 
           && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance); */
        })
      .until(() -> (retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance
                 && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance))
     // .andThen(() -> stopExtend())
      .finallyDo(interrupt -> stopExtend())
      ;
  }

  /*  public CommandBase raisingComand(double Target) {
    return runOnce(() -> raise(raiserEncoder.getPosition() < Target))

    .until(() -> raiserEncoder.getPosition() >= Target - ArmConstants.raiserTolerance 
              && raiserEncoder.getPosition() <= Target + ArmConstants.raiserTolerance)

    .finallyDo(interrupt -> stopRaise())
    ;
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Arm extender position", retractorEncoder.getPosition());
    //SmartDashboard.putNumber("Arm raiser position", raiserEncoder.getPosition());
    //SmartDashboard.putNumber("Arm extenderf position", retractorfollowerEncoder.getPosition());
    //SmartDashboard.putBoolean("am I done?", IamDone);
  }

  /** Display arm motors' parameters on SmartDashBoard */
  public void healthStatus() {
    SmartDashboard.putNumber("Shoulder Current", retractorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder Temp", retractorMotor.getMotorTemperature());
    SmartDashboard.putNumber("Shoulder RPM", retractorEncoder.getVelocity());
    SmartDashboard.putNumber("ShoulderF Current", retractorMotorfollower.getOutputCurrent());
    SmartDashboard.putNumber("ShoulderF Temp", retractorMotorfollower.getMotorTemperature());
    SmartDashboard.putNumber("ShoulderF RPM", retractorfollowerEncoder.getVelocity());
  }

}
