package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private IntakeSubsystem intakE;

    public IntakeCommand(Double inSpeed, Double outSpeed, IntakeSubsystem intake) {
        intakE = intake;
        addRequirements(intakE);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {}
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
