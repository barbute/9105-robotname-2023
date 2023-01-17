//In Java We Trust
/*
 * Note: Don't trust this until encoders and gyro is setup
 *
 * Coded for Xbox Controller
 * A = Arm up
 * B = Arm down
 * X = Grab
 * Y = Release
 */

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private DriveSubsystem drive;
  private IntakeSubsystem intake;
  private Trigger aButton;
  private Trigger bButton;
  private Trigger xButton;
  private Trigger yButton;

  private CommandXboxController Controller;

  SendableChooser<Command> chooser = new SendableChooser<>();


  public RobotContainer() {
    Controller = new CommandXboxController(Constants.controllerPort);
    aButton = Controller.a();
    bButton = Controller.b();
    xButton = Controller.x();
    yButton = Controller.y();

    drive = new DriveSubsystem();
    intake = new IntakeSubsystem();

    drive.setDefaultCommand(new ArcadeCommand(
      () -> Controller.getLeftY(), 
      () -> Controller.getRightX(), 
      drive
      ));

    configureBindings();

    chooser.addOption("Test", loadPathplannerTrajectoryToRamseteCommand(
      "/Users/k2so/Desktop/Mantle/Domino/src/main/deploy/pathplanner/generatedJSON/Test.wpilib.json",
     true));

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    Trajectory trajectory;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    }catch(IOException exception) {
      DriverStation.reportError("Unable to open trajectory: " + filename, exception.getStackTrace());
      System.out.println("Unabe to read from file " + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drive::getPose, 
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMetrer, 
        Constants.kaVoltSecondsSquaredPerMeter), 
      Constants.kDriveKinematics, drive::getWheelSpeeds, 
      new PIDController(Constants.kPDrivelVel, 0, 0), 
      new PIDController(Constants.kPDrivelVel, 0, 0), drive::tankDriveVolts, drive);

      if (resetOdometry) {
        return new SequentialCommandGroup(
          new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())),ramseteCommand);
      }
      else {
        return ramseteCommand;
      }
  }

  private void configureBindings() {
    aButton.onTrue(new InstantCommand(intake::lift, intake));
    bButton.onTrue(new InstantCommand(intake::down, intake));
    xButton.onTrue(new InstantCommand(intake::grab, intake));
    yButton.onTrue(new InstantCommand(intake::release, intake));

  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
