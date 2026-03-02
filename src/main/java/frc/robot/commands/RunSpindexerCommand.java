package frc.robot.commands;

import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class RunSpindexerCommand extends Command{
    @SuppressWarnings("PMD.UnusedPrivateField")

  private final SpindexerSubsystem spindexer;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private double speed;
  private boolean isJammed = false;
  private Timer jamTimer = new Timer();
  public RunSpindexerCommand(SpindexerSubsystem spindexer, double speed) {
    this.spindexer = spindexer;
    this.speed = speed;
    
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
  public void execute() {
    if (spindexer.getSpindexerCurrent().gte(SpindexerConstants.spindexerJamThreshold)) {
      isJammed = true;
      jamTimer.start();
    }
    if (isJammed && jamTimer.hasElapsed(SpindexerConstants.jamRecoveryTime)) {
      isJammed = false;
      jamTimer.stop();
      jamTimer.reset();
    }
    
    if (!isJammed) {
      spindexer.runSpindexer(SpindexerConstants.SPINDEXER_SPEED);
    } 
    else {
      spindexer.runSpindexer(SpindexerConstants.jamRecoverySpeed);
    }
    Logger.recordOutput("Spindexer/Jammed", isJammed);
  }

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
