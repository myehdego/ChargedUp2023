// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.Pneumatics;

public class Gripper extends SubsystemBase {
  /** manipulates and grabs game objects.
   *  opening and closing with pneumatics.
   *  bleeding and closing the pressure switch
   */
  private PneumaticHub pch = new PneumaticHub(1);
  //private Compressor compressor = pch.makeCompressor();
  private DoubleSolenoid gripper; 
  private DoubleSolenoid bleeder;
  private boolean inOrSpit;  // determines direction of the rollers
  private int pieceType;
  private CANSparkMax roller = new CANSparkMax(CANIDs.GripperRollerMotor, MotorType.kBrushless);
  // How we might use a sensor to detect a grabbable game piece
  //private DigitalInput sensor;
  //private SparkMaxLimitSwitch sensor;
  private RelativeEncoder rollerEncoder;

  public Gripper() {
    gripper = new DoubleSolenoid(PneumaticsModuleType.REVPH,
           Pneumatics.openChannel, Pneumatics.closeChannel);
/*     lifter = new DoubleSolenoid(PneumaticsModuleType.REVPH,
          Pneumatics.wristUpChannel, Pneumatics.wristDownChannel); */
    roller.restoreFactoryDefaults();
    roller.setInverted(CANIDs.GripperRollerMotorInverted);
    roller.setIdleMode(IdleMode.kBrake);
    bleeder = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
          Pneumatics.BLEED_CHANNEL_BLEED, Pneumatics.BLEED_CHANNEL_CLOSE);
    inOrSpit = true;
    deBleed();
    //  set these based on info from pressure switch when that info is available on setup
    //pch.enableCompressorAnalog(Pneumatics.CONEPRESSURE-10, Pneumatics.CONEPRESSURE);
    //pieceType=GripperConstants.CONE;

    // How we might use a sensor to detect a grabbable game piece
    //sensor = new DigitalInput(GripperConstants.CONTACTSWITCH_PIN);  // connected to RoboRIO
    //sensor = roller.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);  // connected to a motorController
    //sensor.enableLimitSwitch(true);
    rollerEncoder = roller.getEncoder();
  }

  /** turn on rollers to input a game piece*/
  public void rollersGo() {
    roller.set(GripperConstants.rollerspeed);
  }

  public void rollersStop() {
    roller.stopMotor();
  }

  /** turn on rollers to expel */
  public void rollerSpit() {
    roller.set(-GripperConstants.rollerspeed);
  }

  /**  toggles direction of roller motor
   *   and reports value
   *   false is spit, true is in
   */
  public boolean expel() {
    if (inOrSpit == true) {inOrSpit = false;}
    else inOrSpit = true;
    return inOrSpit;
  }
  
  public void setexpel(boolean spit) {
    inOrSpit = spit;
  }

  /** set gripper strength suitable for cube */
  public void setCubeP() {
    pch.enableCompressorAnalog(Pneumatics.CUBEPRESSURE-10., Pneumatics.CUBEPRESSURE);
    pieceType = GripperConstants.CUBE;
  }

  public void bleed() {
    bleeder.set(Value.kForward);
  }

  public void deBleed() {
    bleeder.set(Value.kReverse);
  }

  /** set gripper strength suitable for cone */
  public void setConeP() {
    pch.enableCompressorAnalog(Pneumatics.CONEPRESSURE-10., Pneumatics.CONEPRESSURE);
    pieceType = GripperConstants.CONE;
  }

  /**config gripper for selected game piece.
   * 
   * @param cone true; else set for cube
  */
  public void initPnGP(boolean cone) {
    if (cone) setConeP();
    else setCubeP();
  }

  /** report the type of game piece that is desired.
   * 
   * @return 
   */
  public int getPieceType()
  {
    return pieceType;
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

  // How we might use a sensor to detect a grabbable game piece
  /** reports whether the sensor has been tripped to indicate
   * something is grabbable
   */
  public boolean grabbable() {
    //return sensor.get();  // test to see which sensor value indicates closed
    //return sensor.isPressed();
    if (rollerEncoder.getVelocity()> -3000)   // -5000 is the unencumbered speed for sucking
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
