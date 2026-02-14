package frc.robot.commands;

import frc.robot.subsystems.kicker.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerConstants;

public class RunKickerCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final KickerSubsystem kicker;
  
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunKickerCommand(KickerSubsystem kicker) {
    this.kicker = kicker;
    
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kicker);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kicker.runKicker(KickerConstants.KICKER_SPEED);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kicker.runKicker(0.5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.runKicker(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
