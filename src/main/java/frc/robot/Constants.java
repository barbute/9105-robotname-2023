//All values are dummy until we get robot built
//Cough Cough Aaron hurry up

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final int leftFrontMotor = 1;
  public static final int leftBackMotor = 2;
  public static final int rightFrontMotor = 3;
  public static final int rightBackMotor = 4;

  public static final int armMotor = 6;
  public static final int grabMotor = 7;

  public static final int controllerPort = 5;

  public static final double ksVolts = 10.0;
  public static final double kvVoltSecondsPerMetrer = 11.1;
  public static final double kaVoltSecondsSquaredPerMeter = 12.2;
  public static final double kPDrivelVel = 13.3;

  public static final double kTrackWidthMetres = Units.inchesToMeters(23);
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
    kTrackWidthMetres);

  public static final double kMaxSpeedMetersPerSecond = 14.4;
  public static final double kMaxAccelerationMetersPerSecondSquared = 15.5;

  public static final double kRamseteB = 16.6;
  public static final double kRamseteZeta = 17.7;

  public static final double kGearRatio = 18.8;
  public static final double kWheelRatioInches = 19.9;

  public static final double kLinearDistanceConversionFactor = (Units
    .inchesToMeters(1/kGearRatio*2*Math.PI*Units.inchesToMeters(kWheelRatioInches)*10));

}
