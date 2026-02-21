// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.util.SendableSupplier;

import static edu.wpi.first.units.Units.Degrees;

import java.time.Instant;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final GenericHID m_operatorController = new GenericHID(
      OperatorConstants.kOperatorControllerPort);

  // private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", autoChooser);

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    Supplier<Double> sliderInput = () -> m_operatorController.getRawAxis(OperatorConstants.slider);
    Supplier<Double> operatorYaw = () -> m_operatorController.getRawAxis(OperatorConstants.yaw);
    Supplier<Double> operatorPitch = () -> m_operatorController.getRawAxis(OperatorConstants.pitch);
    Supplier<Double> operatorRoll = () -> m_operatorController.getRawAxis(OperatorConstants.roll);
    m_driverController.a().onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1)).andThen(new InstantCommand(()-> m_driverController.setRumble(RumbleType.kBothRumble, 0))));

    SmartDashboard.putData("Slider",new SendableSupplier<Double>("Slider", sliderInput));
    SmartDashboard.putData("Yaw",new SendableSupplier<Double>("Yaw", operatorYaw));
    SmartDashboard.putData("Pitch",new SendableSupplier<Double>("Pitch", operatorPitch));
    SmartDashboard.putData("Roll",new SendableSupplier<Double>("Roll", operatorRoll));



// Regenerate tuner constants before doing anything with swerve



    // Regenerate tuner constants before doing anything with swerve

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return autoChooser.getSelected();
  // }
}
