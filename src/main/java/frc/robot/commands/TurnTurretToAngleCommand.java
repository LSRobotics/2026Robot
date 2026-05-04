// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;

import static org.wpilib.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Angle;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret.TurretConstants;
import org.wpilib.command2.Command;

/** An example command that uses an example subsystem. */
public class TurnTurretToAngleCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final TurretSubsystem m_turret;
  private final Supplier<Angle> angle;
  private final PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnTurretToAngleCommand(TurretSubsystem subsystem, Angle angle) {
    m_turret = subsystem;
    this.angle = ()->angle;
    SmartDashboard.putData(pid);
    pid.setTolerance(TurretConstants.tolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

public TurnTurretToAngleCommand(TurretSubsystem subsystem, Supplier<Angle> angle) {
    m_turret = subsystem;
    this.angle = angle;
    SmartDashboard.putData(pid);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double setpoint = MathUtil.clamp(angle.get().in(Degrees), -TurretConstants.turretRangeOneWay.in(Degrees), TurretConstants.turretRangeOneWay.in(Degrees));

    pid.setSetpoint(setpoint);
    double speed = pid.calculate(m_turret.getAngle().in(Degrees));
    
    speed = MathUtil.clamp(speed, -TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed);
    m_turret.setSpeed(speed);
    Logger.recordOutput("Turret/PID_Error", pid.getError());
    Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());
    Logger.recordOutput("Turret/Speed", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() || DriverStation.isDisabled();
  }
}
