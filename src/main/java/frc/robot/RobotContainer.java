// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtHubCommand;
import frc.robot.commands.ArmOutCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOBlinkin;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmLimitSwitchIOLimitSwitch;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunFlywheelCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunKickerCommand;
import frc.robot.commands.RunSpindexerCommand;
import frc.robot.commands.SetHoodAngleCommand;
import frc.robot.commands.ShootAtHubCommand;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterFlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.ShooterHoodIOLinearActuator;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOTalon;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.SendableSupplier;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.commands.TurnTurretToAngleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOSparkFlex;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.logging.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

  private final SpindexerIOSparkFlex SpinIO = new SpindexerIOSparkFlex();
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem(SpinIO);
  private final KickerIO kickerIO = new KickerIOTalonFX();
  private final KickerSubsystem m_kicker = new KickerSubsystem(kickerIO);
  private final TurretIO turretIO = new TurretIOTalon();
  private final TurretSubsystem m_turret = new TurretSubsystem(turretIO);

  private final ShooterSubsystem m_shooter = new ShooterSubsystem(new ShooterFlywheelIOTalonFX(),
      new ShooterHoodIOLinearActuator(ShooterConstants.HoodConstants.hoodLinearActuatorPWMID,
          ShooterConstants.HoodConstants.hoodLinearActuatorPWMID2));

  // Replace with CommandPS4Controller or Commandm_driverController if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  // private final GenericHID m_operatorController = new GenericHID(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final IntakeIOTalonFX intakeIO = new IntakeIOTalonFX();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeIO);

  private final ArmIOSparkMax armIO = new ArmIOSparkMax();
  private final ArmLimitSwitchIOLimitSwitch limitSwitchIO = new ArmLimitSwitchIOLimitSwitch();
  private final ArmSubsystem armSubsystem = new ArmSubsystem(armIO, limitSwitchIO);

  private final LedsIOBlinkin ledIO = new LedsIOBlinkin();
  private final LedSubsystem ledSubsystem = new LedSubsystem(ledIO);

      private double MaxSpeed = 4d;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  private final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();

  private Trigger flywheelAtSpeed = new Trigger(() -> m_shooter.getFlywheelVelocity() != Logger.getLogger("Aiming/Flywheel/TargetRPM"));

  // private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    SmartDashboard.putData("SysId/Flywheel Quasistatic Forward",
        m_shooter.sysIdFlywheelQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Flywheel Quasistatic Reverse",
        m_shooter.sysIdFlywheelQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Flywheel Dynamic Forward",
        m_shooter.sysIdFlywheelDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Flywheel Dynamic Reverse",
        m_shooter.sysIdFlywheelDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    SmartDashboard.putData("Start logger", new InstantCommand(() -> SignalLogger.start()));
    SmartDashboard.putData("Stop logger", new InstantCommand(() -> SignalLogger.stop()));
    

    NamedCommands.registerCommand("ArmOut", new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_DEPLOY_ANGLE));
    NamedCommands.registerCommand("ArmIn", new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_REST_ANGLE));
    NamedCommands.registerCommand("Intake", new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED));
    
    NamedCommands.registerCommand("Shoot", 
        Commands.parallel(new ShootAtHubCommand(m_turret, m_shooter, ledSubsystem, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds), 
                          new ConditionalCommand(new RunKickerCommand(m_kicker, KickerConstants.KICKER_SPEED).andThen(
                                                 new WaitCommand(0.75).andThen(
                                                 new RunSpindexerCommand(spindexer, SpindexerConstants.SPINDEXER_SPEED))), new InstantCommand(), flywheelAtSpeed)).withTimeout(3));

  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_Swerve.setDefaultCommand(
            // m_Swerve will execute this command periodically
            m_Swerve.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                // negative X (left)
            ));

    Supplier<Double> sliderInput = () -> m_operatorController.getRawAxis(OperatorConstants.slider);
    Supplier<Double> operatorYaw = () -> m_operatorController.getRawAxis(OperatorConstants.yaw);
    Supplier<Double> operatorPitch = () -> m_operatorController.getRawAxis(OperatorConstants.pitch);
    Supplier<Double> operatorRoll = () -> m_operatorController.getRawAxis(OperatorConstants.roll);


    SmartDashboard.putData("Slider", new SendableSupplier<Double>("Slider", sliderInput));
    SmartDashboard.putData("Yaw", new SendableSupplier<Double>("Yaw", operatorYaw));
    SmartDashboard.putData("Pitch", new SendableSupplier<Double>("Pitch", operatorPitch));
    SmartDashboard.putData("Roll", new SendableSupplier<Double>("Roll", operatorRoll));

    SmartDashboard.putNumber("speed", 0);
    SmartDashboard.putNumber("Angle", 0);
    DoubleSupplier speedSupplier = () -> SmartDashboard.getNumber("speed", 0);
    DoubleSupplier angleSupplier = () -> SmartDashboard.getNumber("Angle",0);

    // Regenerate tuner constants b[]\efore doing anything with swerve

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on relea[]\se.
    //m_driverController.a().onTrue(new TurnTurretToAngleCommand(m_turret, () -> Degree.of(speedSupplier.getAsDouble())));
    m_driverController.x().whileTrue(new RunSpindexerCommand(spindexer, SpindexerConstants.SPINDEXER_SPEED));
    m_driverController.y().whileTrue(new InstantCommand(()->m_kicker.runKicker(KickerConstants.KICKER_SPEED))).whileFalse(new InstantCommand(()->m_kicker.runKicker(0)));
    m_driverController.povUp().whileTrue(new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_REST_ANGLE));
    m_driverController.povDown().whileTrue(new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_DEPLOY_ANGLE));
    m_driverController.a().whileTrue(new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED));
    m_driverController.leftBumper().onTrue(new InstantCommand((()->m_shooter.setHoodPosition(angleSupplier))));

    m_driverController.b().whileTrue(new RunFlywheelCommand(m_shooter, ()-> RotationsPerSecond.of(speedSupplier.getAsDouble()))).onFalse(new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(0)));
    

    m_operatorController.rightBumper().onTrue(new ArmOutCommand(armSubsystem, nextArmAngle()));
  }

  public Angle nextArmAngle() {
    if (Math.abs(armSubsystem.getArmEncoder().minus(ArmMotorConstants.ARM_DEPLOY_ANGLE).in(Degrees)) < Math.abs(armSubsystem.getArmEncoder().minus(ArmMotorConstants.ARM_REST_ANGLE).in(Degrees))) {
      return ArmMotorConstants.ARM_REST_ANGLE;
    }
    return ArmMotorConstants.ARM_DEPLOY_ANGLE;
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
