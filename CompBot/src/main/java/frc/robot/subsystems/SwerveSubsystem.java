package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            ,"FL"
            );

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
            ,"FR"
            );

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
            ,"BL"
            );

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
            ,"BR"
            );


    private static WPI_Pigeon2 pigeon = new WPI_Pigeon2(1);
///////////////////////////////////////////////////////////////////////////////////////
// Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.

    private final SwerveDriveOdometry odometer = 
        new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                            pigeon.getRotation2d(),
                            getModuleStates(), 
                            new Pose2d(0.0, 0.0, new Rotation2d()));
////////////////////////////////////////////////////////////////////////////////////

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        pigeon.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(pigeon.getAngle(), 360);
       // return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    /** Gets the pose in meters relative to the field coordinate system */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        frontLeft.resetPos();
        frontRight.resetPos();
        backLeft.resetPos();
        backRight.resetPos();
        odometer.resetPosition(new Rotation2d(pigeon.getAngle()),new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),  
            backLeft.getPosition(),
            backRight.getPosition()
        }    ,pose );
    }

    @Override
    public void periodic() {
        odometer.update(new Rotation2d(-pigeon.getAngle()*Math.PI/180.), getModuleStates());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModulePosition [] getModuleStates() {
        return new SwerveModulePosition [] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void reportStatesToSmartDashbd(SwerveModuleState[] desiredStates) {
        frontLeft.smartDashreportState(desiredStates[0]);
        frontRight.smartDashreportState(desiredStates[1]);
        backLeft.smartDashreportState(desiredStates[2]);
        backRight.smartDashreportState(desiredStates[3]);

//        SmartDashboard.putString("Gyro: ", String.format("%.3f", gyro.getAngle()));

        SmartDashboard.putNumber("FLabsA", frontLeft.getabsoluteEncoder());
        SmartDashboard.putNumber("FRabsA", frontRight.getabsoluteEncoder());
        SmartDashboard.putNumber("BLabsA", backLeft.getabsoluteEncoder());
        SmartDashboard.putNumber("BRabsA", backRight.getabsoluteEncoder());

        SmartDashboard.putNumber("BotX", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("BotY", odometer.getPoseMeters().getY());

        SmartDashboard.putNumber("FLPos", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("FRPos", frontRight.getDrivePosition());
        SmartDashboard.putNumber("BLPos", backLeft.getDrivePosition());
        SmartDashboard.putNumber("BRPos", backRight.getDrivePosition());

        SmartDashboard.putNumber("FLTrn", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FRTrn", frontRight.getTurningPosition());
        SmartDashboard.putNumber("BLTrn", backLeft.getTurningPosition());
        SmartDashboard.putNumber("BRTrn", backRight.getTurningPosition());
    }

    /** driveMe just stops because it is unimplemented */
    public CommandBase driveMe(double howfast){
        return runOnce(
            () -> stopModules()
        );
    }
}
