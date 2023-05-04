package frc.robot;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Balancer;
import frc.robot.commands.DriveGeneric;
//import frc.robot.commands.GetAprilTag;
//import frc.robot.commands.GetRobotPosition;
//import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AprilTagCamera;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem;
    private AprilTagCamera camera;
    //private DriveGeneric driveGeneric;
    private WPI_Pigeon2 gyro;
    
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  
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
        swerveSubsystem = new SwerveSubsystem();
        SmartDashboard.putData(swerveSubsystem);
        
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE){
                camera = new AprilTagCamera();
        }
        gyro = new WPI_Pigeon2(1);
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
        
                // mechJoytick Buttons
         
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE) {
                //new JoystickButton(buttonBox1, OIConstants.kgetRobotPositionButton).
                        //onTrue(new GetRobotPosition(camera));
                //new JoystickButton(buttonBox1, OIConstants.kgetAprilTagButton).
                        //onTrue(new GetAprilTag(camera));
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

        //double ySpeed = switchBox.getRawAxis(OIConstants.choiceswitch);
        //double level = switchBox.getRawAxis(OIConstants.levelSwitch);
        //double delay = switchBox.getRawAxis(OIConstants.delaySwitch);

        return new DriveGeneric(swerveSubsystem, FieldConstants.leaveCommunityDist, 0);  // the simplest command
        //return new DriveGeneric(swerveSubsystem, Units.inchesToMeters(48), 0);  // for testing
        //return new AutoPlaceNMove(arm, swerveSubsystem, gripper);       // TODO need mechanism to select this (after testing)
        //return m_chooser.getSelected();
     }

    public Command getAutonomousCommand_old() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                  .setKinematics(old?DriveConstants.kDriveKinematics:DriveConstants.kDriveKinematics);

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
                old?DriveConstants.kDriveKinematics:DriveConstants.kDriveKinematics,
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

    public WPI_Pigeon2 getGyro() {
        return gyro;
    }

    public AprilTagCamera getPhotonVisionSS() {
        if (old?Constants.PHOTONVISION_AVAILABLE:Constants.PHOTONVISION_AVAILABLE) {
                return camera;
        } else return null;
    }

}
