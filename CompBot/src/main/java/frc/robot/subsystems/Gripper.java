// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.Pneumatics;

public class Gripper extends SubsystemBase {
  /** manipulates and grabs game objects.
   *  opening and closing with pneumatics.
   */
  private DoubleSolenoid gripper;  
  private CANSparkMax roller = new CANSparkMax(CANIDs.GripperRollerMotor, MotorType.kBrushless);


  public Gripper() {
    gripper = new DoubleSolenoid(PneumaticsModuleType.REVPH,
           Pneumatics.openChannel, Pneumatics.closeChannel);
    roller.restoreFactoryDefaults();
  }

  public void rollersGo() {
    roller.set(1.);
  }

  public void rollersStop() {
    roller.stopMotor();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
