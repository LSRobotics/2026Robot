package frc.robot.commands;

import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RunIndexerCommand extends Command {
    @SuppressWarnings("PMD.UnusedPrivateField")

  private final SpindexerSubsystem spindexer;

  private final KickerSubsystem kicker;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIndexerCommand(SpindexerSubsystem spindexer, KickerSubsystem kicker) {
    this.spindexer = spindexer;
    this.kicker = kicker;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexer, kicker);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexer.runSpindexer(SpindexerConstants.SPINDEXER_SPEED);
    kicker.runKicker(KickerConstants.KICKER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kicker.runKicker(0.5);
    if (kicker.getKickerSpeed() >= 0.45) {
        spindexer.runSpindexer(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.runSpindexer(0);
    kicker.runKicker(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}