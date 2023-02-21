// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.Pneumatics;

public class Gripper extends SubsystemBase {
  /** manipulates and grabs game objects.
   *  opening and closing with pneumatics.
   *  TODO: change the grip strength for cones and blocks
   */
  private PneumaticHub pch = new PneumaticHub(1);
  private Compressor compressor = pch.makeCompressor();
  private DoubleSolenoid gripper; 
  private DoubleSolenoid lifter; 
  private CANSparkMax roller = new CANSparkMax(CANIDs.GripperRollerMotor, MotorType.kBrushless);

  
  public Gripper() {
    gripper = new DoubleSolenoid(PneumaticsModuleType.REVPH,
           Pneumatics.openChannel, Pneumatics.closeChannel);
    lifter = new DoubleSolenoid(PneumaticsModuleType.REVPH,
          Pneumatics.wristUpChannel, Pneumatics.wristDownChannel);
    roller.restoreFactoryDefaults();
    roller.setInverted(CANIDs.GripperRollerMotorInverted);
    pch.enableCompressorDigital();
  }

  /** turn on rollers */
  public void rollersGo() {
    roller.set(1.);
  }

  public void rollersStop() {
    roller.stopMotor();
  }

  public void setCubeP() {
    pch.enableCompressorAnalog(Pneumatics.CUBEPRESSURE-10., Pneumatics.CUBEPRESSURE);
  }

  public void setConeP() {
    pch.enableCompressorAnalog(Pneumatics.CONEPRESSURE-10., Pneumatics.CONEPRESSURE);
  }
  
  public boolean opengripper() {
    gripper.set(Value.kForward);
    if ( gripper.get() == Value.kForward) return true;
    return false;
  } 

  
  public boolean closegripper() {
    gripper.set(Value.kReverse);
    if ( gripper.get() == Value.kReverse) return true;
    return false;
  } 

  public void liftGripper() {
    lifter.set(Value.kForward);
  }

  public void lowerGripper() {
    lifter.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
