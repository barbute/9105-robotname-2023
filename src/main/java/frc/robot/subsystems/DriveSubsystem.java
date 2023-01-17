/*
 * Notes: Update encoders, gyro, and
 * motor controlers once we get 'em.
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_VictorSPX leftFront;
  private WPI_VictorSPX leftBack;
  private WPI_VictorSPX rightFront;
  private WPI_VictorSPX rightBack;
/* 
  private Encoder leftEncoder;
  private Encoder rightEncoder;
*/
  private DifferentialDrive robotDrive;
  //^
  private final DifferentialDriveOdometry robotOdometry;

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftBack, leftFront);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightBack, rightFront);

  //^
  private static final Gyro navX = new AHRS(SPI.Port.kMXP);

  public DriveSubsystem() {

    leftFront  = new WPI_VictorSPX(Constants.leftFrontMotor);
    leftBack = new WPI_VictorSPX(Constants.leftBackMotor);
    rightFront = new WPI_VictorSPX(Constants.rightFrontMotor);
    rightBack = new WPI_VictorSPX(Constants.rightBackMotor);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    //Add proper encoders here

    //Set encoder position conversion factors here
    //Set encoder velocity conversion factors here

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftMotors.setInverted(true);

    //^
    navX.reset();
    //^
    navX.calibrate();
    //^
    resetEncoders();

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    //Replace 0's with encoders
    //^
    robotOdometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
    //^
    robotOdometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getHeading(), new Pose2d());
  }

  public void resetEncoders() {
    //rightEncoder.setPosition(0);
    //leftEncoder.setPosition(0);
  }

  public void arcadeDrive(double speed, double rotation) {
    robotDrive.arcadeDrive(speed, rotation);
  }

  //^
  public double getRightEncoderPosition() {
    //return rightEncoder.getPosition();
    return 1.1;
  }
  
  //^
  public double getLeftEncoderPosition() {
    //return leftEncoder.getPosition();
    return 1.1;
  }

  //^
  public double getRightEncoderVelocity() {
    //return rightEncoder.getVelocity();
    return 1.1;
  }

  //^
  public double getLeftEncoderVelocity() {
    //return -leftEncoder.getVelocity();
    return 1.1;
  }

  //^
  public double getTurnRate() {
    //return -navX.getRate();
    return 1.1;
  }

  //^
  public static double getHeading() {
    //return navX.getRotation2d().getDegrees();
    return 1.1;
  }

  //^
  public Pose2d getPose() {
    return robotOdometry.getPoseMeters();
  }

  //^
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    robotOdometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getHeading(), pose);
  }

  //^
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  //^
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    robotDrive.feed();
  }

  //^
  public double averageEncoderDistance() {
    return ((getRightEncoderPosition() + getLeftEncoderPosition()) / 2.0);
  }

  //Create get encoder methods here

  //^
  public void setMaxOutput(double MaxOutput) {
    robotDrive.setMaxOutput(MaxOutput);
  }

  //^
  public static void zeroHeading() {
    navX.reset();
    navX.calibrate();
  }

  //^
  public Gyro getGyro() {
    return getGyro();
  }

  @Override
  public void periodic() {
    //Replace 0's with encoders
    //^
    robotOdometry.update(navX.getRotation2d(), 0, 0);

    //^
    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());

  }
}
