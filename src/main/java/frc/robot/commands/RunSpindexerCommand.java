package frc.robot.commands;

import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
public class RunSpindexerCommand extends Command{
    @SuppressWarnings("PMD.UnusedPrivateField")

  private final SpindexerSubsystem spindexer;
  //private SparkFlex m_motor; 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunSpindexerCommand(SpindexerSubsystem spindexer) {
    this.spindexer = spindexer;
    //m_motor = new SparkFlex(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spindexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexer.runSpindexer(SpindexerConstants.SPINDEXER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.runSpindexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
