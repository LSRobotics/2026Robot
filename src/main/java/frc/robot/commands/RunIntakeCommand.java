// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LedSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntakeCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeSubsystem m_intakeSubsystem;
  private final LedSubsystem m_Leds;

  private final DoubleSupplier speed;

  /**
   * Creates a new RunIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeCommand(IntakeSubsystem intake, LedSubsystem led, double speed) {
    m_intakeSubsystem = intake;
    m_Leds = led;
    this.speed = () -> speed;

    addRequirements(intake);
  }

  public RunIntakeCommand(IntakeSubsystem intake, LedSubsystem led, DoubleSupplier speed) {
    m_intakeSubsystem = intake;
    m_Leds = led;
    this.speed = speed;


    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.runIntake(speed.getAsDouble());
    if (speed.getAsDouble() < 0){
      m_Leds.setColor(LEDConstants.colorWhite);
    } else if (speed.getAsDouble() > 0) {
      m_Leds.setColor(LEDConstants.colorOrange);
    }
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
