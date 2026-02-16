// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntakeCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new RunIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeCommand(IntakeSubsystem intake) {
    m_intakeSubsystem = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.runIntake(IntakeConstants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
