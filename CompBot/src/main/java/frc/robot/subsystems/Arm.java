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
    // raiserMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.raiserForwardLimit);  // TODO set me
    // raiserMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.raiserReverseLimit);
    // raiserMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // raiserMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  /** disable soft limit for the reverse direction of the retractor motor
   * ( for use during set up -- during test mode )
   */
  // TODO: does raiser motor need softlimit disabled?
  public boolean softLimitONOFF() {
    if (retractorMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse)) 
      retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    else retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    return retractorMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
  }
  
  /** raise forarm */
  public void raise() {
    raiserMotor.set(0.5);
  }
  /** raise forarm on true,
   *  lower on false
  */
  public void raise(boolean direction) {
    if(direction) raise();
    else lower();
  }

  /** lower forarm */
  public void lower() {
    raiserMotor.set(-0.5);
  } 

  /** report status of Arm Commands */
  public boolean amIDone() {
    return IamDone;
  }

  /* enforce completion of Arm Commands */
  public void makeMeDone() {
    IamDone = true;
  }

  /** update P parameter for retractor motor closed loop controller. */
  public void pidCoefficient(double distance, double raiserdistance) {
    kp = 1 * .4 / distance;
    rkp = 1 * .4 / distance;
    retractorPidController.setP(kp);
    raiserPidController.setP(rkp);
  }

  double latestTargetE;  // target position for retractor
  double latestTargetR;  // target position for raiser
  /** run retractor motor closed loop controller */
  public void closedLoopController(double Target, double RaiserTarget) {
    IamDone = false;
    latestTargetE = Target;
    latestTargetR = RaiserTarget;
    retractorPidController.setReference(Target, ControlType.kPosition);
    raiserPidController.setReference(RaiserTarget, ControlType.kPosition);
  }

  /**  Adjusts the target for the retractor pidcontroller */
  public void retargetRetract(double Adjustment) {
    latestTargetE += Adjustment;
    retractorPidController.setReference(latestTargetE, ControlType.kPosition);
  }

  /** Adjust the target for the raiser pidcontroller */
  public void retargetRaise(double Adjustment) {
    latestTargetR += Adjustment;
    retractorPidController.setReference(latestTargetR, ControlType.kPosition);
  }

  /** extend the upper arm  */
  public void extend() {
    retractorMotor.set(0.5);
  }
  /** Extends if boolean "direction" is true, retracts if false */
  public void extend(boolean direction) {
    if(direction) extend();
    else retract();
  }

  /** retract the upper arm */
  public void retract() {
    retractorMotor.set(-0.5);
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
              SmartDashboard.putNumber("target", Target);
        }) 
      .andThen(() -> {
        //extend();
              SmartDashboard.putBoolean("currentPosTest", 
              retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance 
           && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance);
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
    SmartDashboard.putNumber("Arm extender position", retractorEncoder.getPosition());
    //SmartDashboard.putNumber("Arm extenderf position", retractorfollowerEncoder.getPosition());
    SmartDashboard.putBoolean("am I done?", IamDone);
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
