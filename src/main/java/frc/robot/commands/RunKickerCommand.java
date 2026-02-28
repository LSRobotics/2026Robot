// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunKickerCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final KickerSubsystem kicker;
  private final double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunKickerCommand(KickerSubsystem kicker, double speed) {
    this.kicker = kicker;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kicker.runKicker(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
