// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmLimitSwitchConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmOutCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ArmSubsystem m_arm;
  private final double angle;
  private double tolerance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmOutCommand(ArmSubsystem arm, double angle) {
    m_arm = arm;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { // TODO SIGNS ARE NOT FINAL; MUST BE TUNED FOR DIRECTION
    if (m_arm.getArmEncoder().in(Degrees) < angle) {
        tolerance = -ArmMotorConstants.ARM_TOLERANCE;
        m_arm.runArm(ArmMotorConstants.ARM_SPEED);
    } else {
        tolerance = ArmMotorConstants.ARM_TOLERANCE;
        m_arm.runArm(-ArmMotorConstants.ARM_SPEED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
    m_arm.setAngle(Angle.ofBaseUnits(0, Degrees));
  }

  // Returns true when the command should end.
  @SuppressWarnings("unlikely-arg-type")
  @Override
  public boolean isFinished() {
    return m_arm.getArmEncoder().in(Degrees) == (angle - tolerance) || m_arm.getLimitSwitch();
  }
}

