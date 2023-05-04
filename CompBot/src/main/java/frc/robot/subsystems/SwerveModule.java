package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderFaults;
//import com.ctre.phoenix.sensors.CANCoderStickyFaults;
//import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final SparkMaxPIDController turningPidController;
    //private final PIDController wturningPidController;

    private final WPI_CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    private String reportName="";

    public SwerveModule(int driveMotorId, int turningMotorId, 
            boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed,
            String name) {
        this(driveMotorId, turningMotorId, 
            driveMotorReversed, turningMotorReversed,
            absoluteEncoderId, absoluteEncoderReversed);
        this.reportName = name;
    }
    public SwerveModule(int driveMotorId, int turningMotorId, 
        boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, boolean absoluteEncoderReversed,
        boolean comp) {
        this(driveMotorId, turningMotorId, 
            driveMotorReversed, turningMotorReversed,
            absoluteEncoderId, absoluteEncoderReversed);
    }
    public SwerveModule(int driveMotorId, int turningMotorId, 
        boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, boolean absoluteEncoderReversed,
        String name,
        boolean comp) {
        this(driveMotorId, turningMotorId, 
            driveMotorReversed, turningMotorReversed,
            absoluteEncoderId, absoluteEncoderReversed);
        this.reportName=name;
    }

    public SwerveModule(int driveMotorId, int turningMotorId, 
            boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new WPI_CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(
            ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(
            ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(
            ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(
            ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = turningMotor.getPIDController();
        turningPidController.setP( 2.0);
        turningPidController.setI( 0.);
        turningPidController.setFF( 0.);
        turningPidController.setD( 0.);
        turningPidController.setOutputRange(-1., 1.);
        turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
        turningPidController.setPositionPIDWrappingMinInput(-Math.PI);
        turningPidController.setPositionPIDWrappingEnabled(true);
        //wturningPidController = new PIDController(2., 0., 0.);
        //wturningPidController.enableContinuousInput(0., Math.PI*2.);
        resetEncoders();
    }

    public SwerveModulePosition getPosition() {   
      return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    } 

    /** Returns drive position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /** returns drive velocity in meters/sec */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
    public double getabsoluteEncoder()
    {
        return absoluteEncoder.getAbsolutePosition();
    }
    
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();    // [0 - 360]
        angle *= Math.PI /180.;     // [ 0 - 2PI]
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    public double getDriveTemperature() {
        return driveMotor.getMotorTemperature(); 
    }
    public double getTurningTemperature() {
        return turningMotor.getMotorTemperature(); 
    }
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public void resetPos() {
        driveEncoder.setPosition(0.);
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /** sets drive motor idle mode to COAST */
    public void setcoastmode() {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    /** sets drive motor idle mode to BRAKE */
    public void setbrakemode() {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / 
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningPidController.setReference(state.angle.getRadians(),ControlType.kPosition);
        //double turnControl = wturningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        //turningMotor.set(Math.max(-1.,Math.min(1.,turnControl)));
        //SmartDashboard.putNumberArray("Swerve[" + absoluteEncoder.getDeviceID() + "] turn", new double[]{turnControl,turningMotor.get()});
    }

    public void smartDashreportState(SwerveModuleState state) {
        SmartDashboard.putNumber(reportName+ " ABS Encoder" ,absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(reportName+ " Encoder" ,turningEncoder.getPosition());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
