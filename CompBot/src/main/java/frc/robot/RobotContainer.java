package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmRun;
import frc.robot.commands.DriveGeneric;
import frc.robot.commands.DriverStation;
import frc.robot.commands.GetAprilTag;
import frc.robot.commands.GetRobotPosition;
import frc.robot.commands.GripperOpenClose;
import frc.robot.commands.GripperUpAndDown;
import frc.robot.commands.NudgeDrive;
//import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.AprilTagCamera;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem;
    private DriveGeneric driveGeneric;
    private Arm arm;
    private final Joystick switchBox =  new Joystick(OIConstants.kDriverControllerPort4);
    
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  //  private final Joystick buttonBox = new Joystick(OIConstants.kDRiverCOntrollerPort2);
  //  private final Joystick buttonBox2 = new Joystick(OIConstants.kDRiverCOntrollerPort3);

    
    private final Joystick buttonBox0 = new Joystick(OIConstants.kButtonBoxPort_0);
    private final Joystick buttonBox1 = new Joystick(OIConstants.kButtonBoxPort_1);


    private Gripper gripper;
    private AprilTagCamera camera;
    boolean old;  // true if original swerve constants
    
    public RobotContainer(boolean Old) {
        /*  swapped out to put drive function in teleopPeriodic
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        */
        this.old=Old;
        swerveSubsystem = new SwerveSubsystem(Old);
        SmartDashboard.putData(swerveSubsystem);
        
        if (old?Constants.ARM_AVAILABLE:Constants.ARM_AVAILABLE_Comp) arm = new Arm();
        if (old?Constants.GRIPPER_AVAILABLE:Constants.GRIPPER_AVAILABLE_Comp) gripper = new Gripper();
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE_Comp) camera = new AprilTagCamera();
        configureButtonBindings();
    }
    public RobotContainer() {
        this(true);
    }

    private void configureButtonBindings() { 
        // driverJoytick Buttons
        new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).
          onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoytick, OIConstants.kDriverResetOdometryButtonIdx).
          onTrue(new InstantCommand(() -> 
          swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0)))));
        // whenPressed(() -> swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0))));
        // TODO test this drive stuff
        /* new JoystickButton(buttonBox1, OIConstants.kDrivertostationbutton).
                onTrue(new DriverStation(swerveSubsystem)); */
        /* new JoystickButton(driverJoytick, OIConstants.kNudgeLeftButton).
                onTrue(new NudgeDrive(swerveSubsystem, 0, 0)); */
        
        // Test commands for generic drive
  /* 
        new JoystickButton(buttonBox, OIConstants.kEndDriveGeneric).
                onTrue(new InstantCommand(() -> swerveSubsystem.makemestop()));
        new JoystickButton(buttonBox, OIConstants.kDriveGenericx).
                onTrue(new InstantCommand(() -> swerveSubsystem.makemestop()).
                andThen(new WaitCommand(.5)).
                andThen(driveGeneric = new DriveGeneric(swerveSubsystem, Units.inchesToMeters(14), 0)));
        new JoystickButton(buttonBox, OIConstants.kDriveGenericy).
                onTrue(new InstantCommand(() -> swerveSubsystem.makemestop()).
                andThen(new WaitCommand(.5)).
                andThen(driveGeneric = new DriveGeneric(swerveSubsystem, 0, Units.inchesToMeters(14))));
        new JoystickButton(buttonBox, OIConstants.kDriveGenericxy).
                onTrue(new InstantCommand(() -> swerveSubsystem.makemestop()).
                andThen(new WaitCommand(.5)).
                andThen(driveGeneric = new DriveGeneric(swerveSubsystem, Units.inchesToMeters(14), Units.inchesToMeters(14))));
 */       
                // mechJoytick Buttons
         if (old?Constants.ARM_AVAILABLE:Constants.ARM_AVAILABLE_Comp) { 
                new JoystickButton(buttonBox0, OIConstants.kFloorPos_BB0).   //floor level
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new InstantCommand(() -> arm.setFloor(true))).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.floorPosition,ArmConstants.floorPositionR,true)));
                new JoystickButton(buttonBox0, OIConstants.kSubStationPos_BB0).   //substation 
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new InstantCommand(() -> arm.setFloor(false))).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.substation,ArmConstants.substationR,true)));
                new JoystickButton(buttonBox0, OIConstants.kMidGridPos_BB0).   //mid level
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new InstantCommand(() -> arm.setFloor(false))).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.cubeDepth1,ArmConstants.cubeDepth1R,true)));
                new JoystickButton(buttonBox0, OIConstants.kHighGridPos_BB0).     //high level
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new InstantCommand(() -> arm.setFloor(false))).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.cubeDepth2,ArmConstants.cubeDepth2R,true)));
                new JoystickButton(buttonBox0, OIConstants.kRetractPos_BB0).     //retract
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.retracto0,ArmConstants.retracto0R,true)));
/*                 new JoystickButton(buttonBox1, OIConstants.kArmDone).
                  onTrue(new InstantCommand(() -> arm.makeMeDone())); */
                new JoystickButton(buttonBox1, OIConstants.targetExtendNudge_BB1).
                  onTrue(new InstantCommand(() -> arm.retargetRetract(ArmConstants.targetRetractorNudgeamount)));
                new JoystickButton(buttonBox1, OIConstants.targetRetractNudge_BB1).
                  onTrue(new InstantCommand(() -> arm.retargetRetract(-ArmConstants.targetRetractorNudgeamount)));
                new JoystickButton(buttonBox1, OIConstants.targetRaiseNudge_BB1).
                  onTrue(new InstantCommand(() -> arm.retargetRaise(ArmConstants.targetRaiseNudgeamount)));
                new JoystickButton(buttonBox1, OIConstants.targetLowerNudge_BB1).
                  onTrue(new InstantCommand(() -> arm.retargetRaise(-ArmConstants.targetRaiseNudgeamount)));
        
                // onTrue(arm.extensionCommand(ArmConstants.cubeDepth2));
                // onTrue(arm.extensionCommand(ArmConstants.cubeDepth1));
        }

        if (old?Constants.GRIPPER_AVAILABLE:Constants.GRIPPER_AVAILABLE_Comp){
                new JoystickButton(buttonBox1, OIConstants.kgripperopenbutton_BB1).
                  onTrue(new GripperOpenClose(gripper, true));
                new JoystickButton(buttonBox1, OIConstants.kgripperclosebutton_BB1).
                    onTrue(new GripperOpenClose(gripper, false));
                // TODO: open requires high pressure
                // TODO: need to bleed pressure when switching from high to low
        
                new JoystickButton(buttonBox1, OIConstants.PRESSURESwitch_BB1).
                    onTrue(new InstantCommand(() -> gripper.setCubeP()));
                new JoystickButton(buttonBox1, OIConstants.PRESSURESwitch_BB1).
                    onFalse(new InstantCommand(() -> gripper.setConeP()));
        } 
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE_Comp) {
                new JoystickButton(buttonBox1, OIConstants.kgetRobotPositionButton).
                        onTrue(new GetRobotPosition(camera));
                new JoystickButton(buttonBox1, OIConstants.kgetAprilTagButton).
                        onTrue(new GetAprilTag(camera));
        }
     }
    
     public Command getAutonomousCommand() {
        /* 
         * Autonomous selector dial selects command group to follow:
         * 1. goes stright forward to leave the Community
         * 2. place a game piece on a node.
         * 3. retriving a game piece.
         * 4. docking on the charge station.
         * 5. 2 followed by 1
         * 6. 2 followed by 4
         * 7. 1 followed by 4
         * 8.
         * 
         * Presuming one step is to play a game piece, 
         * there is a selector for wihich location
         * 1. floor
         * 2. mid
         * 3. high
         * 
         * Presuming we want to allow alliance partner(s) to get
         * out of the way before we drive, there is a selector
         * to choose the delay time, 0 -> 5 seconds
         */
        /*  TODO: convert axis output from [-1, 1] to:
            command set choice [1,N]
            level choice [floor, mid, high]
            delay [0,5]
        */

        /* TODO: Do any of our comptations or decisions depend on which color we are?
            Answer is Yes
        */

        /* TODO: What is our location in the GRID? */

        double ySpeed = switchBox.getRawAxis(OIConstants.choiceswitch);
        double level = switchBox.getRawAxis(OIConstants.levelSwitch);
        double delay = switchBox.getRawAxis(OIConstants.delaySwitch);

        return new DriveGeneric(swerveSubsystem, 0, 0);  // TODO where to go?
     }

    public Command getAutonomousCommand_old() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                  .setKinematics(old?DriveConstants.kDriveKinematics:DriveConstants.kDriveKinematics_Comp);

        // 2. Generate trajectory
        double scale = -.4;

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(

                        // new Translation2d(  -1 * scale,   0 * scale),
                        // new Translation2d(  -1 * scale,   1 * scale),
                        // new Translation2d(  -2 * scale ,  1 * scale),
                        // new Translation2d(  -2 * scale ,  0 * scale),
                        // new Translation2d(  -1 * scale,   0 * scale),
                        // new Translation2d(  -1 * scale,   1 * scale),
                        // new Translation2d(   0 * scale,   1 * scale),
                        // new Translation2d(   0 * scale,   0 * scale),

                     //   new Translation2d(  -1 * scale,   0 * scale),
                     //   new Translation2d(  -1 * scale,   1 * scale),
                     //   new Translation2d(  -2 * scale ,  1 * scale),
                     //   new Translation2d(  -2 * scale ,  0 * scale),
                     //   new Translation2d(  -1 * scale,   0 * scale),
                        new Translation2d(  -1 * scale,   1 * scale),
                        new Translation2d(   0 * scale,   1 * scale)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),  //360)),
                trajectoryConfig);



        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                old?DriveConstants.kDriveKinematics:DriveConstants.kDriveKinematics_Comp,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public SwerveSubsystem getSwerveSS() {
            return swerveSubsystem;
    }

    public Arm getarmSS() {
        if (old?Constants.ARM_AVAILABLE:Constants.ARM_AVAILABLE_Comp) {
                return arm;
        } else return null;
    }

    public Gripper getGripperSS() {
        if (old?Constants.GRIPPER_AVAILABLE:Constants.GRIPPER_AVAILABLE_Comp) {
                return gripper;
        } else return null;
    }

    public AprilTagCamera getPhotonVisionSS() {
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE_Comp) {
                return camera;
        } else return null;
    }

}
