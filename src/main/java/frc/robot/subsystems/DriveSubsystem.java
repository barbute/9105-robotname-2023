//Should work maybe :)

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  private DifferentialDrive robotDrive;
  private final DifferentialDriveOdometry robotOdometry;

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftBack, leftFront);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightBack, rightFront);

  
  private static final Gyro navX = new AHRS(SPI.Port.kMXP);

  public DriveSubsystem() {

    leftFront = new CANSparkMax(Constants.leftFrontMotor, 
      CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBack = new CANSparkMax(Constants.leftBackMotor, 
      CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.rightFrontMotor, 
      CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBack = new CANSparkMax(Constants.rightBackMotor, 
      CANSparkMaxLowLevel.MotorType.kBrushless);

    leftFront.setIdleMode(IdleMode.kBrake);

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

    leftEncoder.setPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
    rightEncoder.setPositionConversionFactor(Constants.kLinearDistanceConversionFactor);

    leftEncoder.setVelocityConversionFactor(Constants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(Constants.kLinearDistanceConversionFactor);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftMotors.setInverted(true);

    navX.reset();
    navX.calibrate();
    resetEncoders();

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    //Replace 0's with encoders
    //^
    robotOdometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    robotOdometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getHeading(), new Pose2d());
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void arcadeDrive(double speed, double rotation) {
    robotDrive.arcadeDrive(speed, rotation);
  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }
  
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return -leftEncoder.getVelocity();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public static double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return robotOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    robotOdometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getHeading(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public double averageEncoderDistance() {
    return ((getRightEncoderPosition() + getLeftEncoderPosition()) / 2.0);
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public void setMaxOutput(double MaxOutput) {
    robotDrive.setMaxOutput(MaxOutput);
  }

  public static void zeroHeading() {
    navX.reset();
    navX.calibrate();
  }

  public Gyro getGyro() {
    return getGyro();
  }

  @Override
  public void periodic() {
    robotOdometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());

  }
}
