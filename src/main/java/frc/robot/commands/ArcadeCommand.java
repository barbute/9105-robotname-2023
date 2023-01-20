
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DoubleSupplier speed;
  private DoubleSupplier rotation;
  private DriveSubsystem arcadeDrive;

  public ArcadeCommand(DoubleSupplier Speed, DoubleSupplier Rotation, DriveSubsystem drive) {
    arcadeDrive = drive;
    speed = Speed;
    rotation = Rotation;

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arcadeDrive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
