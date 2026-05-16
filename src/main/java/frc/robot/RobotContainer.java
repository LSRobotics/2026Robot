// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmOutCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOBlinkin;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmLimitSwitchIOLimitSwitch;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.RunFlywheelCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunKickerCommand;
import frc.robot.commands.RunSpindexerCommand;
import frc.robot.commands.SetHoodAngleCommand;
import frc.robot.commands.TakeShotCommand;
import frc.robot.commands.TurnTurretCommand;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterFlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.ShooterHoodIOLinearActuator;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOTalon;
import frc.robot.subsystems.Turret.TurretSubsystem;

import frc.robot.util.ManualFlywheelSpeed;
import frc.robot.util.SendableSupplier;
import frc.robot.util.ShotSolution;
import frc.robot.util.Telemetry;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.commands.TurnTurretToAngleCommand;

import frc.robot.commands.TakeShotCommand.ShotData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOSparkFlex;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    private final SpindexerIOTalonFX SpinIO = new SpindexerIOTalonFX();
    private final SpindexerSubsystem spindexer = new SpindexerSubsystem(SpinIO);
    private final KickerIO kickerIO = new KickerIOTalonFX();
    private final KickerSubsystem m_kicker = new KickerSubsystem(kickerIO);
    private final TurretIO turretIO = new TurretIOTalon();
    private final TurretSubsystem m_turret = new TurretSubsystem(turretIO);

    private final SendableChooser<Command> autoChooser;

    private final ShooterSubsystem m_shooter = new ShooterSubsystem(new ShooterFlywheelIOTalonFX(),
            new ShooterHoodIOLinearActuator(ShooterConstants.HoodConstants.hoodLinearActuatorPWMID,
                    ShooterConstants.HoodConstants.hoodLinearActuatorPWMID2));

    // Replace with CommandPS4Controller or Commandm_driverController if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    // private final GenericHID m_operatorController = new
    // GenericHID(OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    private final IntakeIOTalonFX intakeIO = new IntakeIOTalonFX();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeIO);

    private final ArmIOSparkMax armIO = new ArmIOSparkMax();
    private final ArmLimitSwitchIOLimitSwitch limitSwitchIO = new ArmLimitSwitchIOLimitSwitch();
    private final ArmSubsystem armSubsystem = new ArmSubsystem(armIO, limitSwitchIO);

    private final LedsIOBlinkin ledIO = new LedsIOBlinkin();
    private final LedSubsystem ledSubsystem = new LedSubsystem(ledIO);

    private double leftRightTrim = 0.0;

    private Double MaxSpeed = Constants.maxSpeedSlow;
    private Double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // Use
                                                                                                           // open-loop
                                                                                                           // control
                                                                                                           // for
                                                                                                           // drive
                                                                                                           // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();

    private Trigger flywheelAtSpeed = new Trigger(
            () -> (m_shooter.getFlywheelVelocity().minus(m_shooter.targetSpeed)
                    .abs(RotationsPerSecond)) <= (ShooterConstants.FlywheelConstants.flywheelTolerance
                            .in(RotationsPerSecond)));

    DoubleSupplier opRightX = () -> m_operatorController.getRightX();
    DoubleSupplier opLeftY = () -> m_operatorController.getLeftY();
    Trigger opRJoystickX = new Trigger(() -> opRightX.getAsDouble() != 0);
    Trigger opLJoystickY = new Trigger(() -> opLeftY.getAsDouble() != 0);


    public RobotContainer() {
        LEDManager.init(ledSubsystem);
        LEDManager.setDefault();



        // Configure the trigger bindings

        // autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();


        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        SmartDashboard.putData("Start logger", new InstantCommand(() -> SignalLogger.start()));
        SmartDashboard.putData("Stop logger", new InstantCommand(() -> SignalLogger.stop()));

        NamedCommands.registerCommand("ArmOut",
                new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_DEPLOY_ANGLE).withTimeout(Seconds.of(20)));
        NamedCommands.registerCommand("ArmIn", new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_REST_ANGLE));
        NamedCommands.registerCommand("Intake",
                new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED).withTimeout(3));

        NamedCommands.registerCommand("Spindexer",
                new RunSpindexerCommand(spindexer, SpindexerConstants.SPINDEXER_SPEED).withTimeout(6));
        NamedCommands.registerCommand("Kicker",
                new RunKickerCommand(m_kicker, KickerConstants.KICKER_SPEED).withTimeout(8));

        NamedCommands.registerCommand("LongSpindexer",
                new RunSpindexerCommand(spindexer, SpindexerConstants.SPINDEXER_SPEED).withTimeout(11));
        NamedCommands.registerCommand("LongKicker",
                new RunKickerCommand(m_kicker, KickerConstants.KICKER_SPEED).withTimeout(11));

        NamedCommands.registerCommand("Print", new PrintCommand("Print"));

        NamedCommands.registerCommand("LongIntake",
                new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED).withTimeout(10));

        NamedCommands.registerCommand("LongishIntake",
                new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED).withTimeout(3.7));

        NamedCommands.registerCommand("ShootFromLeftBump",
                new TakeShotCommand(m_turret, m_shooter, TakeShotCommand.ShotData.leftBump).withTimeout(4));

        NamedCommands.registerCommand("ShootFromLeftTrench",
                 new TakeShotCommand(m_turret, m_shooter, TakeShotCommand.ShotData.leftTrench).withTimeout(4));

        NamedCommands.registerCommand("LongShootFromLeftTrench",
                new TakeShotCommand(m_turret, m_shooter, TakeShotCommand.ShotData.leftTrench).withTimeout(10));

        NamedCommands.registerCommand("ShootFromRightTrench",
                new TakeShotCommand(m_turret, m_shooter, TakeShotCommand.ShotData.rightTrench).withTimeout(4));

        NamedCommands.registerCommand("LongShootFromRightTrench",
                new TakeShotCommand(m_turret, m_shooter, TakeShotCommand.ShotData.rightTrench).withTimeout(10));

        NamedCommands.registerCommand("AimFromTrench", 
                new PrintCommand("Removed fro demos"));

        NamedCommands.registerCommand("LeftFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed)).withTimeout(4));

        NamedCommands.registerCommand("LongLeftFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed)).withTimeout(10));

        NamedCommands.registerCommand("RightFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed)).withTimeout(4));

        NamedCommands.registerCommand("LongRightFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed)).withTimeout(10));

        NamedCommands.registerCommand("FullVision", 
                new PrintCommand("Removed fro demos"));

        NamedCommands.registerCommand("LongFullVision", 
                new PrintCommand("Removed fro demos"));

        NamedCommands.registerCommand("FeedFromNeutral", 
                new PrintCommand("Removed fro demos"));


        NamedCommands.registerCommand("CenterShoot", new PrintCommand("Removed fro demos"));
        NamedCommands.registerCommand("Shoot3sec",
                new PrintCommand("Removed fro demos"));

        NamedCommands.registerCommand("Shoot6sec",
                new PrintCommand("Removed fro demos"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);

        Logger.recordOutput("LeftRightTrim", leftRightTrim);
    }

        private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        m_Swerve.setDefaultCommand(
                // m_Swerve will execute this command periodically
                m_Swerve.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                              // with
                // negative X (left)
                ));

        m_Swerve.registerTelemetry(new Telemetry(MaxSpeed)::telemeterize);

        SmartDashboard.putNumber("speed", 0);
        SmartDashboard.putNumber("Angle", 0);
        DoubleSupplier speedSupplier = () -> SmartDashboard.getNumber("speed", 0);
        DoubleSupplier angleSupplier = () -> SmartDashboard.getNumber("Angle", 0);

        m_driverController.x().whileTrue(
                new RunKickerCommand(m_kicker, KickerConstants.KICKER_SPEED)
                        .alongWith(
                                Commands.sequence(
                                        new RunSpindexerCommand(
                                                spindexer,
                                                SpindexerConstants.jamRecoverySpeed).withTimeout(0.25),
                                        new RunSpindexerCommand(
                                                spindexer,
                                                SpindexerConstants.SPINDEXER_SPEED))));

        m_driverController.povUp().whileTrue(new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_REST_ANGLE));
        m_driverController.povDown().whileTrue(new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_DEPLOY_ANGLE));
        m_driverController.a()
                .whileTrue(new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED));
        m_driverController.povRight().onTrue(new InstantCommand((() -> m_shooter.setHoodPosition(angleSupplier))));

        opRJoystickX.and(m_operatorController.start().negate()).whileTrue(new TurnTurretCommand(m_turret, opRightX));

        opLJoystickY.whileTrue(
                new ConditionalCommand(
                        new ArmOutCommand(armSubsystem, () -> ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE, 
                                () -> Math.abs(opLeftY.getAsDouble()) * ArmMotorConstants.ARM_SPEED),
                        new ArmOutCommand(armSubsystem, () -> ArmConstants.ArmMotorConstants.ARM_REST_ANGLE, 
                                () -> Math.abs(opLeftY.getAsDouble()) * ArmMotorConstants.ARM_SPEED),
                        () -> opLeftY.getAsDouble() > 0d));

        // m_driverController.povLeft()
        // .whileTrue(Commands.parallel(new RunKickerCommand(m_kicker,
        // KickerConstants.KICKER_SPEED),
        // new WaitCommand(0.75).andThen(new RunSpindexerCommand(spindexer,
        // SpindexerConstants.SPINDEXER_SPEED))));

         m_driverController.b()
                 .whileTrue(new RunFlywheelCommand(m_shooter, () -> RotationsPerSecond.of(speedSupplier.getAsDouble())))
                 .onFalse(new InstantCommand(()-> m_shooter.setFlywheelVoltage(Volt.of(0))));

        // m_operatorController.rightTrigger().whileTrue(
        // new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, ()
        // -> m_Swerve.getState().Speeds));

        m_driverController.start()
                .onTrue(new InstantCommand(() -> m_Swerve.resetRotation(
                        DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(180)
                                : Rotation2d.fromDegrees(0))));

        m_operatorController.povUp()
                .onTrue(new InstantCommand(() -> ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed1)));
        m_operatorController.povRight()
                .onTrue(new InstantCommand(() -> ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed2)));
        m_operatorController.povLeft()
                .onTrue(new InstantCommand(() -> ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed3)));
        m_operatorController.povDown()
                .onTrue(new InstantCommand(() -> ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed4)));

        m_operatorController.y()
                .whileTrue(Commands.parallel(
                        new WaitCommand(0.5)
                                .andThen(new TurnTurretToAngleCommand(m_turret, TurretConstants.manualAngle1)),
                        new RunFlywheelCommand(m_shooter, () -> ManualFlywheelSpeed.getSpeed())));
        m_operatorController.x()
                .whileTrue(Commands.parallel(
                        new WaitCommand(0.5)
                                .andThen(new TurnTurretToAngleCommand(m_turret, TurretConstants.manualAngle2)),
                        new RunFlywheelCommand(m_shooter, () -> ManualFlywheelSpeed.getSpeed())));
        m_operatorController.b().and(m_operatorController.a().negate())
                .whileTrue(Commands.parallel(
                        new WaitCommand(0.5)
                                .andThen(new TurnTurretToAngleCommand(m_turret, TurretConstants.manualAngle3)),
                        new RunFlywheelCommand(m_shooter, () -> ManualFlywheelSpeed.getSpeed())));

        m_operatorController.rightBumper()
                .whileTrue(Commands.parallel(new RunFlywheelCommand(m_shooter, () -> ManualFlywheelSpeed.getSpeed())));



        m_driverController.y().whileTrue(Commands.run(() -> {m_shooter.setHoodPosition(-1); LEDManager.setColor(LEDConstants.colorGold);}, m_shooter).withName("Hood Down").withInterruptBehavior(InterruptionBehavior.kCancelIncoming)).onFalse(Commands.runOnce(LEDManager::setDefault));

        m_operatorController.leftTrigger()
                .whileTrue(new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.OUTTAKE_SPEED));
    }

    public Angle nextArmAngle() {
        if (Math.abs(armSubsystem.getArmEncoder().minus(ArmMotorConstants.ARM_DEPLOY_ANGLE).in(Degrees)) < Math
                .abs(armSubsystem.getArmEncoder().minus(ArmMotorConstants.ARM_REST_ANGLE).in(Degrees))) {
            return ArmMotorConstants.ARM_REST_ANGLE;
        }
        return ArmMotorConstants.ARM_DEPLOY_ANGLE;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
    
    public void periodic() {
        SmartDashboard.putBoolean("FlywheelAtSpeed", flywheelAtSpeed.getAsBoolean());
        if (Math.random()<0.01){
                Runtime.getRuntime().gc();
        }
        Logger.recordOutput("Swerve Current",
                m_Swerve.getModules()[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
    }
}

