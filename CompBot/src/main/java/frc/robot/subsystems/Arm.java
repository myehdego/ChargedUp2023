// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
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
  private SparkMaxPIDController pidController;
  private RelativeEncoder retractorEncoder;
  private RelativeEncoder retractorfollowerEncoder;
  private RelativeEncoder raiserEncoder;
  double kp;
  private boolean IamDone;
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
    pidController = retractorMotor.getPIDController();
    retractorEncoder.setPositionConversionFactor(ArmConstants.retractorEncoderScale);  //  degrees
    retractorfollowerEncoder.setPositionConversionFactor(ArmConstants.retractorEncoderScale);  //  degrees
    raiserEncoder = raiserMotor.getEncoder(); 
    raiserEncoder.setPositionConversionFactor(ArmConstants.raiserEncoderScale);  //  degrees
    retractorMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.retractorForwardLimit);  // TODO set me
     retractorMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.retractorReverseLimit);
     retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
     retractorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // raiserMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.raiserForwardLimit);  // TODO set me
    // raiserMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.raiserReverseLimit);
    retractorMotor.setIdleMode(IdleMode.kCoast);
    retractorMotorfollower.setIdleMode(IdleMode.kCoast);
  }

  public boolean softLimitONOFF() {
    if (retractorMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse)) 
      retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    else retractorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    return retractorMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
  }
  
   public void raise() {
    raiserMotor.set(0.1);
  }
  public void raise(boolean direction) {
    if(direction) raise();
    else lower();
  }

  public void lower() {
    raiserMotor.set(-0.1);
  } 

  public boolean amIDone() {
    return IamDone;
  }

  public void makeMeDone() {
    IamDone = true;
  }

  public void pidCoefficient(double distance) {
    kp = 1 * .4 / distance;
    pidController.setP(kp);
  }

  public void closedLoopController(double Target) {
    IamDone = false;
    pidController.setReference(Target, ControlType.kPosition);
  }

  public void extend() {
    retractorMotor.set(0.5);
  }
  /** Extends if boolean "direction" is true, retracts if false */
  public void extend(boolean direction) {
    if(direction) extend();
    else retract();
  }

  public void retract() {
    retractorMotor.set(-0.5);
  }

  public void stopRaise() {
    raiserMotor.stopMotor();
  }
  public void stopExtend() {
    retractorMotor.stopMotor();
  }

  public double getRaiserPO() {
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

  /** This runs open looped controller, will command for arm to extend or retract towards target */
  public CommandBase extensionCommand(double Target) {
    return runOnce(() -> {extend(retractorEncoder.getPosition() < Target);
                         SmartDashboard.putNumber("target", Target);}) 

      .andThen(() -> {
        //extend();
        SmartDashboard.putBoolean("currentPosTest", retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance 
        && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance);
      })

      .until(() -> (retractorEncoder.getPosition() >= Target - ArmConstants.retractorTolerance
                    && retractorEncoder.getPosition() <= Target + ArmConstants.retractorTolerance))

     // .andThen(() -> stopExtend())

      .finallyDo(interrupt -> stopExtend())
      ;
  }

  /* public CommandBase raisingComand(double Target) {
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
    SmartDashboard.putNumber("Arm extenderf position", retractorfollowerEncoder.getPosition());
    SmartDashboard.putBoolean("am I done?", IamDone);
  }

  public void healthStatus() {
    SmartDashboard.putNumber("Shoulder Current", retractorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder Temp", retractorMotor.getMotorTemperature());
    SmartDashboard.putNumber("Shoulder RPM", retractorEncoder.getVelocity());
    SmartDashboard.putNumber("ShoulderF Current", retractorMotorfollower.getOutputCurrent());
    SmartDashboard.putNumber("ShoulderF Temp", retractorMotorfollower.getMotorTemperature());
    SmartDashboard.putNumber("ShoulderF RPM", retractorfollowerEncoder.getVelocity());
  }

}
