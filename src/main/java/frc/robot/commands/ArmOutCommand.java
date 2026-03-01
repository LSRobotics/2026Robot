// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmOutCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ArmSubsystem m_arm;
  private final Angle targetAngle;
  private double targetArmDegrees;
  private double targetMotorDegrees;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmOutCommand(ArmSubsystem arm, Angle angle) {
    m_arm = arm;
    this.targetAngle = angle;
    this.targetArmDegrees = angle.in(Degrees);
    this.targetMotorDegrees = targetArmDegrees * ArmMotorConstants.gearRatio;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Arm/TargetArmDegrees", targetArmDegrees);
    Logger.recordOutput("Arm/TargetMotorDegrees", targetMotorDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motorDegrees = m_arm.getArmEncoder().in(Degrees);
    double armDegrees = motorDegrees / ArmMotorConstants.gearRatio;
    double armErrorDegrees = targetArmDegrees - armDegrees;
    double motorErrorDegrees = targetMotorDegrees - motorDegrees;

    Logger.recordOutput("Arm/MotorAngleDegrees", motorDegrees);
    Logger.recordOutput("Arm/ArmAngleDegrees", armDegrees);
    Logger.recordOutput("Arm/TargetArmDegrees", targetArmDegrees);
    Logger.recordOutput("Arm/TargetMotorDegrees", targetMotorDegrees);
    Logger.recordOutput("Arm/ArmErrorDegrees", armErrorDegrees);
    Logger.recordOutput("Arm/MotorErrorDegrees", motorErrorDegrees);

    if (Math.abs(armErrorDegrees) <= ArmMotorConstants.ARM_TOLERANCE) {
      m_arm.runArm(0);
      return;
    }

    double direction = Math.signum(motorErrorDegrees);

    // if (direction > 0 && m_arm.getLimitSwitch()) {
    // m_arm.runArm(0);
    // return;
    // }

    m_arm.runArm(direction * ArmMotorConstants.ARM_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double armDegrees = m_arm.getArmEncoder().in(Degrees) / ArmMotorConstants.gearRatio;
    return Math.abs(targetArmDegrees - armDegrees) <= ArmMotorConstants.ARM_TOLERANCE;
  }
}
