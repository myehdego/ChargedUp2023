package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import java.io.ObjectInputStream.GetField;
import java.text.ChoiceFormat;

import com.ctre.phoenix.sensors.Pigeon2_Faults;

// }

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    // CRG added stuff to drive in teleopPeriodic rather than defaultCommand {
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private SwerveSubsystem swerveSubsystem;
    private Arm arm;
    private WPI_Pigeon2 pigeon;

    private PowerDistribution PDP;
    private PowerDistribution PDH;
    private AnalogInput pixyCam;
    private AnalogInput systemChooser; // Value of a hard wired constant resistor to determine which robo
    private boolean choice;  // choose which robot to control, true is competion bot
    // }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.
        //      o build subsystems based on what is available,
        //      o perform all our button bindings,
        //      o put our autonomous chooser on the dashboard.

        choice = Preferences.containsKey("Swerve");
        //systemChooser = new AnalogInput(Constants.SYSTEMCHOOSER);
        //choice = systemChooser.getValue() == Constants.COMPBOT;
        if (choice) {
            PDH = new PowerDistribution(1, ModuleType.kRev);
        } else {
            PDP = new PowerDistribution(1, ModuleType.kCTRE);
        }
       
        System.out.println("PDP = " + PDP.getType());
        // PDH = new PowerDistribution(1, ModuleType.kRev);
        if (Constants.PIXY_AVAILABLE) pixyCam = new AnalogInput(0);
        m_robotContainer = new RobotContainer(!choice);
        // TODO: Get Information about CompBot Swerve Modules
        pigeon = new WPI_Pigeon2(1);

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // CRG added stuff to drive in teleopPeriodic rather than defaultCommand {
        this.swerveSubsystem = m_robotContainer.getSwerveSS();
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        arm = m_robotContainer.getarmSS();
        // arm.resetEncoders();
        // }
    }

    double smoothedXSpeed = 0.;
    double smoothedYSpeed = 0.;
    double smoothedTurningSpeed = 0.;
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        if (driverJoytick.getRawButton(3))
            {
                System.out.println("Button Pressed");
            }

        SmartDashboard.putNumber("BotA", pigeon.getAngle());
        SmartDashboard.putNumber("BatV", PDP.getVoltage());

        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        SmartDashboard.putNumber("Yaw", pigeon.getYaw());
        SmartDashboard.putNumber("Angle", pigeon.getAngle());

        //SmartDashboard.putNumber()

        // CRG use SwerveSubsystem drive methods similar to the SwerveJoysickCmd  {
        // 1. Get real-time joystick inputs
        double xSpeed = driverJoytick.getRawAxis(OIConstants.kDriverXAxis); // Negative values go forward
        double ySpeed = -driverJoytick.getRawAxis(OIConstants.kDriverYAxis);
        double turningSpeed =  -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis);
        //    Smooth driver inputs
        smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .08;
        smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .08;
        smoothedTurningSpeed = smoothedTurningSpeed + (turningSpeed - smoothedTurningSpeed) * .08;
        //    System.out.println("Raw Joystick Values");
        //    System.out.println("X: " + String.format("%.3f", xSpeed) 
        //                    + " Y: " + String.format("%.3f", ySpeed)
        //                    + " R: " + String.format("%.3f", turningSpeed));
        if (Constants.PIXY_AVAILABLE) {
            if (driverJoytick.getRawButton(OIConstants.PixyFollowButton)){
                int err = pixyCam.getAverageValue();
                SmartDashboard.putNumber("PixyX",  pixyCam.getAverageValue());
                //double turnSpeedB = (err<1500)?.1:(err>1700?-0.1:0.);
                turningSpeed = Math.max(-.3,
                                Math.min(.3,
                                (err-1600)*-3.e-4  )); 
                smoothedTurningSpeed = turningSpeed; // We're not smoothing yet
                SmartDashboard.putNumber("turnSpeed",smoothedTurningSpeed);
            }
        }

        // 2. Apply deadband
        xSpeed = Math.abs(smoothedXSpeed) > OIConstants.kDeadband ? smoothedXSpeed : 0.0;
        ySpeed = Math.abs(smoothedYSpeed) > OIConstants.kDeadband ? smoothedYSpeed : 0.0;
        turningSpeed = Math.abs(smoothedTurningSpeed) > OIConstants.kDeadband ? smoothedTurningSpeed : 0.0;
        //System.out.println("Deadband Applied");
        //System.out.println("X: " + String.format("%.3f", xSpeed)
        //                + " Y: " + String.format("%.3f", ySpeed)
        //                + " R: " + String.format("%.3f", turningSpeed));

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        //System.out.println("Smoothing Applied");
        //System.out.println("X: " + String.format("%.3f", xSpeed)
        //                + " Y: " + String.format("%.3f", ySpeed)
        //                + " R: " + String.format("%.3f", turningSpeed));

        //System.out.println("=====================");

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if ( !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        //System.out.println("Chassis Speeds");
        //System.out.println("X: " + String.format("%.3f", xSpeed)
        //                + " Y: " + String.format("%.3f", ySpeed)
        //                + " R: " + String.format("%.3f", swerveSubsystem.getRotation2d()));

        //System.out.println("Encoder: " + frontleftsteerencoder.getPosition());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = 
           choice?DriveConstants.kDriveKinematics_Comp.toSwerveModuleStates(chassisSpeeds):
                  DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        /* this should be done in the SwerveSubsystem
        SwerveModuleState[] moduleStates = swerveSubsystem.chassis2ModuleStates(chassisSpeeds);
        */

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
        //}
    
        swerveSubsystem.reportStatesToSmartDashbd(moduleStates);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        arm = m_robotContainer.getarmSS();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        if (driverJoytick.getRawButton(1)){  // A Button
            arm.extend();
        }
        if (driverJoytick.getRawButton(4)){  // Y Button
            arm.retract();
        }
        if (driverJoytick.getRawButton(2)){  // B Button
            arm.stopExtend();
        }
        if (driverJoytick.getRawButton(3)) {  // X button
            arm.resetEncoders();
        }
        
        SmartDashboard.putNumber("Arm extender position", arm.getExtenderPos()); 
    }
}
