package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/* Table of Contents:
 *    Availablility of Subsystems
 *    ModuleConstants
 *    DriveConstants
 *    CANIDs
 *    ArmConstants
 *    AutoConstants
 *    OIConstants
 *    CamConstant
 *    Pneumatics
 *    RobotConstants
 *    FieldConstants
 */

public final class Constants {
  public static final boolean GRIPPER_AVAILABLE = false;
  public static final boolean ARM_AVAILABLE = false;
  public static final boolean PHOTONVISION_AVAILABLE = true;
  public static final boolean PIXY_AVAILABLE = true;
  public static final boolean GRIPPER_AVAILABLE_Comp = false;
  public static final boolean ARM_AVAILABLE_Comp = true;
  public static final boolean PHOTONVISION_AVAILABLE_Comp = false;
  public static final boolean PIXY_AVAILABLE_Comp = false;
  //public static final int SYSTEMCHOOSER = 0;


    public static final class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
      public static final double kDriveMotorGearRatio = 1 / 6.75;
      public static final double kTurningMotorGearRatio = 1 / 12.8;
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 42;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 42;
      public static final double kPTurning = 0.5;
      public static final double kWheelDiameterMeters_Comp = Units.inchesToMeters(3.75);
      public static final double kDriveMotorGearRatio_Comp = 1 / 6.75;
      public static final double kTurningMotorGearRatio_Comp = 1 / 12.8;
     // public static final double kDriveEncoderRot2Meter_Comp = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
     // Theoraticlly maybe; in practice from measurements
      public static final double kDriveEncoderRot2Meter_Comp = Units.inchesToMeters(1)/41.2;
      public static final double kTurningEncoderRot2Rad_Comp = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec_Comp = kDriveEncoderRot2Meter / 42;
      public static final double kTurningEncoderRPM2RadPerSec_Comp = kTurningEncoderRot2Rad / 42;
      public static final double kPTurning_Comp = 0.5;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(21.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.75);
        // Distance between front and back wheels
        public static final double kTrackWidth_Comp = Units.inchesToMeters(16.5);
        //
        public static final double kWheelBase_Comp = Units.inchesToMeters(22.5);
        // 
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
  //              new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
    //            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //          new Translation2d(-kWheelBase / 2,  kTrackWidth / 2));

                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2));

        public static final SwerveDriveKinematics kDriveKinematics_Comp = new SwerveDriveKinematics(
//                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
  //              new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
    //            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //          new Translation2d(-kWheelBase / 2,  kTrackWidth / 2));
                  
                new Translation2d( kWheelBase_Comp / 2, -kTrackWidth_Comp / 2),
                new Translation2d( kWheelBase_Comp / 2,  kTrackWidth_Comp / 2),
                new Translation2d(-kWheelBase_Comp / 2, -kTrackWidth_Comp / 2),
                new Translation2d(-kWheelBase_Comp / 2,  kTrackWidth_Comp / 2));
                  

//    COMP BOT  FRONT                PRAC BOT    FRONT
//     +----------------------+        +----------------------+
//     | D11 S21      D12 S22 |        | D15 S25      D16 S26 |
//     | E31          E32     |        | E35          E36     |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     | D13 S23      D14 S24 |        | D17 S27      D18 S28 |
//     | E33          E34     |        | E37          E38     |
//     +----------------------+        +----------------------+
//
        public static final int kFrontLeftDriveMotorPort        = 11;    // Module 1
        public static final int kFrontLeftTurningMotorPort      = 21;
        public static final int kFrontLeftAbsoluteEncoderPort   = 31;

        public static final int kFrontRightDriveMotorPort       = 12;    // Module 2
        public static final int kFrontRightTurningMotorPort     = 22;
        public static final int kFrontRightAbsoluteEncoderPort  = 32;

        public static final int kBackRightDriveMotorPort        = 13;    // Module 3
        public static final int kBackRightTurningMotorPort      = 23;
        public static final int kBackRightAbsoluteEncoderPort   = 33;

        public static final int kBackLeftDriveMotorPort         = 14;    // Module 4
        public static final int kBackLeftTurningMotorPort       = 24;
        public static final int kBackLeftAbsoluteEncoderPort    = 34;

        public static final int kFrontLeftDriveMotorPort_Comp        = 12;    // Module 1
        public static final int kFrontLeftTurningMotorPort_Comp      = 22;
        public static final int kFrontLeftAbsoluteEncoderPort_Comp   = 32;

        public static final int kFrontRightDriveMotorPort_Comp       = 11;    // Module 2
        public static final int kFrontRightTurningMotorPort_Comp     = 21;
        public static final int kFrontRightAbsoluteEncoderPort_Comp  = 31;

        public static final int kBackRightDriveMotorPort_Comp        = 14;    // Module 3
        public static final int kBackRightTurningMotorPort_Comp      = 24;
        public static final int kBackRightAbsoluteEncoderPort_Comp   = 34;

        public static final int kBackLeftDriveMotorPort_Comp         = 13;    // Module 4
        public static final int kBackLeftTurningMotorPort_Comp       = 23;
        public static final int kBackLeftAbsoluteEncoderPort_Comp    = 33;

        public static final boolean kFrontLeftTurningEncoderReversed  = true;
        public static final boolean kBackLeftTurningEncoderReversed   = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed  = true;

        public static final boolean kFrontLeftDriveEncoderReversed  = false;
        public static final boolean kBackLeftDriveEncoderReversed   = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed  = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed  = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed   = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed  = true;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 1;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kPhysicalMaxSpeedMetersPerSecond_Comp = 1;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond_Comp = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /1.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                                           kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.8;
  
    //    CAN_Number = new int[10];
    //    CAN_Name   = new string[30];

     //   CAN_Number[1] = 11;
     //   CAN_Name[1]   = "Front Left Drive Motor";
     //   CAN_Number[2] = 21;
     //   CAN_Name[2]   = "Front Left Turn Motor";
     //   CAN_Number[3] = 31;
     //   CAN_Name[3]   = "Front Left Absolute Encoder";

      //  CAN_Number[4] = 12;
      //  CAN_Name[4]   = "Front Right Drive Motor";
      //  CAN_Number[5] = 22;
      //  CAN_Name[5]   = "Front Right Turn Motor";
     //   CAN_Number[6] = 32;
      //  CAN_Name[6]   = "Front Right Absolute Encoder";

     //   CAN_Number[7] = 13;
     //   CAN_Name[7]   = "Back Right Drive Motor";
     //   CAN_Number[8] = 23;
     //   CAN_Name[8]   = "Back Right Turn Motor";
     //   CAN_Number[9] = 33;
     //   CAN_Name[9]   = "Back Right Absolute Encoder";

      //  CAN_Number[10] = 14;
     //   CAN_Name[10]   = "Back Left Drive Motor";
     //   CAN_Number[11] = 24;
     //   CAN_Name[11]   = "Back Left Turn Motor";
     //   CAN_Number[12] = 34;
    //    CAN_Name[12]   = "Back Left Absolute Encoder";

      //  CAN_Number[13] = 99;

    }

    public static final class CANIDs {
      public static final int ArmRetractorMotor         = 60;
      public static final int ArmRetractorMotorfollower = 31;
      public static final boolean retractorMotorInverted = true;
      public static final int ArmRaiserMotor            = 42;
      public static final boolean ArmRaiserMotorInverted = false;
      public static final int GripperRollerMotor        = 0;
      public static final boolean GripperRollerMotorInverted = false;
    }

    // Relative to front of the grid 
    public static final class ArmConstants {
      public static final double retractorTolerance = 3.;  
      public static final double raiserTolerance = 3.;  
      public static final double floorPosition = 0.;
      public static final double coneHeight1 = 34.;
      public static final double coneHeight2 = 46.;
      public static final double cubeDepth1 = 175.2; // 22.75
      public static final double cubeDepth2 = 405.6; // 39.75
      public static final double cubeHeight1 = 23.5;
      public static final double cubeHeight2 = 35.5;
      public static final double retracto0 = 0.;
      public static final double coneDepth1 = 22.75;
      public static final double coneDepth2 = 39.75;  // All in inches
      public static final double RETRACTOR_METERSPERCOUNT = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond/1 ;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = .0;   // 1.5;
        public static final double kPYController = .0;    // 1.5;
        public static final double kPThetaController = .0;  //.06; // 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0; // driverJoytick
        public static final int kDRiverCOntrollerPort2 = 1; // mechJoytick
        public static final int kDriverYAxis = 0;
        public static final int kDriverXAxis = 1;
        public static final int kDriverRotAxis = 4;
        public static final int kArmExtendPos0Button          = 1; // mechJoytick button A
        public static final int kArmExtendPos1Button          = 2; // mechJoytick button B
        public static final int kArmExtendPos2Button          = 3; // mechJoytick button X
        public static final int kArmDone                      = 4; // mechJoytick button Y
        public static final int kgetAprilTagButton            = 5; // mechJoytick button left-bumper
        public static final int kgetRobotPositionButton       = 6; // mechJoystick button right-bumper
        public static final int kgripperopenbutton            = 14;

        public static final int kDriverResetGyroButtonIdx     = 1; // driverJoytick button A
        public static final int kDrivertostationbutton        = 2; // driverJoytick button B
        public static final int kDriverResetOdometryButtonIdx = 3; // driverJoytick button X
        public static final int kDriverFieldOrientedButtonIdx = 5; // driverJoytick button left-bumper
        public static final int PixyFollowButton              = 6; // driverJoytick button right-bumper
        
        public static final double kDeadband = 0.05;
        public static final int kgripperliftbutton            = 0;
        public static final int kgripperclosebutton           = 0;
        public static final int kgripperdownbutton            = 0;
    }

    public static final class CamConstant {
        public static final double PitchAngle =           0.; // Pitch angle of camera in degrees
        public static final double CameraLocationX =      0.; // camera location relative to robots center
        public static final double CameraLocationY =      0.;
        public static final double CameraLocationZ =      0.;
        public static final double CameraLocationX_COMP = 0.; // camera location relative to robots center
        public static final double CameraLocationY_COMP = 0.;
        public static final double CameraLocationZ_COMP = 0.;
    }
    public static final class Pneumatics {
        public static final int openChannel = 0;      // the gripper channel to open
        public static final int closeChannel = 1;     // the gripper channel to close
        public static final int wristUpChannel = 2;   // gripper wrist up
        public static final int wristDownChannel = 3; // gripper wrist down
    }

    public static final class RobotConstants {
      public static final double xcg = Units.inchesToMeters(0);
    }

    public static final class FieldConstants {
      public static final double chargingstationwidth = Units.inchesToMeters(48.+13.11*2); // Width of the charging station.
    }

    public static int COMPBOT;
}



/* +---------------------------------------------------------------------------------+
   |  Absolute Encoder Offsets (Degrees) these values are set in the CANcoders and   |
   |                                  are recorded here for backup purposes          |
   |    Module 0  Front Left    -20.83                                               |
   |           1  Back Left    -188.17                                               |
   |           2  Front Right  -287.40                                               |
   |           3  Back Right   -281.33                                               |
   +---------------------------------------------------------------------------------+  */
