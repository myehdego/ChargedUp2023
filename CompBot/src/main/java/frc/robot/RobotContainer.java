package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import frc.robot.commands.DriverStation;
import frc.robot.commands.GetAprilTag;
import frc.robot.commands.GetRobotPosition;
import frc.robot.commands.GripperOpenClose;
import frc.robot.commands.GripperUpAndDown;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.AprilTagCamera;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem;
    private Arm arm;
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick = new Joystick(OIConstants.kDRiverCOntrollerPort2);
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
        new JoystickButton(driverJoytick, OIConstants.kDrivertostationbutton).
                onTrue(new DriverStation(swerveSubsystem));
        // mechJoytick Buttons
         if (old?Constants.ARM_AVAILABLE:Constants.ARM_AVAILABLE_Comp) {
                new JoystickButton(mechJoytick, OIConstants.kArmExtendPos1Button).
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.cubeDepth1,true)));
                new JoystickButton(mechJoytick, OIConstants.kArmExtendPos2Button).
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.cubeDepth2,true)));
                new JoystickButton(mechJoytick, OIConstants.kArmExtendPos0Button).
                  onTrue(new InstantCommand(() -> arm.makeMeDone()).
                  andThen(new WaitCommand(.5)).
                  andThen(new ArmRun(arm,ArmConstants.retracto0,true)));
                new JoystickButton(mechJoytick, OIConstants.kArmDone).
                  onTrue(new InstantCommand(() -> arm.makeMeDone()));
        
                // onTrue(arm.extensionCommand(ArmConstants.cubeDepth2));
                // onTrue(arm.extensionCommand(ArmConstants.cubeDepth1));
        }
        if (old?Constants.GRIPPER_AVAILABLE:Constants.GRIPPER_AVAILABLE_Comp){
                new JoystickButton(mechJoytick, OIConstants.kgripperopenbutton).
                  onTrue(new GripperOpenClose(gripper, true));
                new JoystickButton(mechJoytick, OIConstants.kgripperclosebutton).
                    onTrue(new GripperOpenClose(gripper, false));
        
                new JoystickButton(mechJoytick, OIConstants.kgripperliftbutton).
                  onTrue(new GripperUpAndDown(gripper, true));
                new JoystickButton(mechJoytick, OIConstants.kgripperdownbutton).
                    onTrue(new GripperUpAndDown(gripper, false));
        }
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE_Comp) {
                new JoystickButton(mechJoytick, OIConstants.kgetRobotPositionButton).
                        onTrue(new GetRobotPosition(camera));
                new JoystickButton(mechJoytick, OIConstants.kgetAprilTagButton).
                        onTrue(new GetAprilTag(camera));
        }
     }
    

    public Command getAutonomousCommand() {
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
