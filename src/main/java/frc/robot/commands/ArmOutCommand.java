// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmLimitSwitchConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmOutCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ArmSubsystem m_arm;
  private final Angle angle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmOutCommand(ArmSubsystem arm, Angle angle) {
    m_arm = arm;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Arm motor angle", m_arm.getArmEncoder().in(Degrees));
    Logger.recordOutput("Arm angle", m_arm.getArmEncoder().times(1d/ArmMotorConstants.gearRatio).in(Degrees));
    Logger.recordOutput("Target angle", angle.in(Degrees));
    if (m_arm.getArmEncoder().times(1d/ArmMotorConstants.gearRatio).lt(angle)) {
        m_arm.runArm(ArmMotorConstants.ARM_SPEED);
    } else {
        m_arm.runArm(-ArmMotorConstants.ARM_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(((m_arm.getArmEncoder().times(1d/ArmMotorConstants.gearRatio)).minus(angle)).in(Degrees)) <= ArmMotorConstants.ARM_TOLERANCE;
  }
}

