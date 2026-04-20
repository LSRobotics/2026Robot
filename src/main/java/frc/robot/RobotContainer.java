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
import frc.robot.commands.AimAtHubCommand;
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
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.FeedCommand2;
import frc.robot.commands.RunFlywheelCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunKickerCommand;
import frc.robot.commands.RunSpindexerCommand;
import frc.robot.commands.SetHoodAngleCommand;
import frc.robot.commands.ShootAtHubCommand;
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
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.util.ManualFlywheelSpeed;
import frc.robot.util.SendableSupplier;
import frc.robot.util.ShotSolution;
import frc.robot.util.Telemetry;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.commands.TurnTurretToAngleCommand;
import frc.robot.commands.VisionFixedSpeedCommand;
import frc.robot.commands.WheelRadiusCommand;
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
    private static final double TRIM_DEGREES = 10.0d;

    private Supplier<Double> MaxSpeed = () -> Constants.maxSpeedSlow;
    private Supplier<Double> MaxAngularRate = () -> RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.get() * 0.1).withRotationalDeadband(MaxAngularRate.get() * 0.1) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // Use
                                                                                                           // open-loop
                                                                                                           // control
                                                                                                           // for
                                                                                                           // drive
                                                                                                           // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();

    private final VisionSubsystem m_Vision = new VisionSubsystem(m_Swerve::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.camera0name, VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(VisionConstants.camera1name, VisionConstants.robotToCamera1),
            new VisionIOPhotonVision(VisionConstants.camera2name, VisionConstants.robotToCamera2));

    private Trigger flywheelAtSpeed = new Trigger(
            () -> (m_shooter.getFlywheelVelocity().minus(m_shooter.targetSpeed)
                    .abs(RotationsPerSecond)) <= (ShooterConstants.FlywheelConstants.flywheelTolerance
                            .in(RotationsPerSecond)));

    DoubleSupplier opRightX = () -> m_operatorController.getRightX();
    DoubleSupplier opLeftY = () -> m_operatorController.getLeftY();
    Trigger opRJoystickX = new Trigger(() -> opRightX.getAsDouble() != 0);
    Trigger opLJoystickY = new Trigger(() -> opLeftY.getAsDouble() != 0);

    // private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        LEDManager.init(ledSubsystem);
        LEDManager.setDefault();
        Logger.recordOutput("Swerve/BrakeMode", false);
        Logger.recordOutput("Swerve/BrakeModeColor", "#000000");
        Logger.recordOutput("Swerve/MaxSpeed", MaxSpeed.get());

        ManualFlywheelSpeed.init();
        SmartDashboard.putData("Commands/WheelRadius", new WheelRadiusCommand(m_Swerve));

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

        SmartDashboard.putData("SysId1/Swerve Quasistatic Forward",
                m_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId1/Swerve Quasistatic Reverse",
                m_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysId1/Swerve Dynamic Forward",
                m_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId1/Swerve Dynamic Reverse",
                m_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

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
                new AimAtHubCommand(m_turret, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds).withTimeout(0.2));

        NamedCommands.registerCommand("LeftFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed)).withTimeout(4));

        NamedCommands.registerCommand("LongLeftFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed)).withTimeout(10));

        NamedCommands.registerCommand("RightFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed)).withTimeout(4));

        NamedCommands.registerCommand("LongRightFlywheelFromTrench", 
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed)).withTimeout(10));

        NamedCommands.registerCommand("FullVision", 
                new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds).withTimeout(4));

        NamedCommands.registerCommand("LongFullVision", 
                new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds).withTimeout(10));

        NamedCommands.registerCommand("FeedFromNeutral", 
                new FeedCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds).withTimeout(6)); 


        NamedCommands.registerCommand("CenterShoot", new VisionFixedSpeedCommand(m_turret, m_shooter, new ShotSolution(-1,52,-1), ()->m_Swerve.getState().Pose, ()->m_Swerve.getState().Speeds));
        NamedCommands.registerCommand("Shoot3sec",
                Commands.parallel(
                        new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose,
                                () -> m_Swerve.getState().Speeds),
                        new ConditionalCommand(new RunKickerCommand(m_kicker, KickerConstants.KICKER_SPEED).andThen(
                                new WaitCommand(0.75).andThen(
                                        new RunSpindexerCommand(spindexer, SpindexerConstants.SPINDEXER_SPEED))),
                                new InstantCommand(), flywheelAtSpeed))
                        .withTimeout(3));

        NamedCommands.registerCommand("Shoot6sec",
                Commands.parallel(
                        new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose,
                                () -> m_Swerve.getState().Speeds),
                        new RunSpindexerCommand(spindexer, SpindexerConstants.SPINDEXER_SPEED)
                                .alongWith(new RunKickerCommand(m_kicker, KickerConstants.KICKER_SPEED)))
                        .withTimeout(6));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);

        Logger.recordOutput("LeftRightTrim", leftRightTrim);
    }

        private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(m_exampleSubsystem));

        m_Swerve.setDefaultCommand(
                // m_Swerve will execute this command periodically
                m_Swerve.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed.get()) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed.get()) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate.get()) // Drive counterclockwise
                                                                                              // with
                // negative X (left)
                ));

        m_Swerve.registerTelemetry(new Telemetry(MaxSpeed.get())::telemeterize);

        // Supplier<Double> sliderInput = () ->
        // m_operatorController.getRawAxis(OperatorConstants.slider);
        // Supplier<Double> operatorYaw = () ->
        // m_operatorController.getRawAxis(OperatorConstants.yaw);
        // Supplier<Double> operatorPitch = () ->
        // m_operatorController.getRawAxis(OperatorConstants.pitch);
        // Supplier<Double> operatorRoll = () ->
        // m_operatorController.getRawAxis(OperatorConstants.roll);

        // SmartDashboard.putData("Slider", new SendableSupplier<Double>("Slider",
        // sliderInput));
        // SmartDashboard.putData("Yaw", new SendableSupplier<Double>("Yaw",
        // operatorYaw));
        // SmartDashboard.putData("Pitch", new SendableSupplier<Double>("Pitch",
        // operatorPitch));
        // SmartDashboard.putData("Roll", new SendableSupplier<Double>("Roll",
        // operatorRoll));

        SmartDashboard.putNumber("speed", 0);
        SmartDashboard.putNumber("Angle", 0);
        DoubleSupplier speedSupplier = () -> SmartDashboard.getNumber("speed", 0);
        DoubleSupplier angleSupplier = () -> SmartDashboard.getNumber("Angle", 0);

        // Regenerate tuner constants b[]\efore doing anything with swerve

        // Schedule `exampleMethodCommand` when the Xbox controller's B []\button is
        // pressed,
        // []\cancelling on relea[]\se.
        // m_driverController.a().onTrue(new TurnTurretToAngleCommand(m_turret, () ->
        // Degree.of(speedSupplier.getAsDouble())));
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
        // m_driverController.y().whileTrue(new InstantCommand(() ->
        // m_kicker.runKicker(KickerConstants.KICKER_SPEED)))
        // .whileFalse(new InstantCommand(() -> m_kicker.runKicker(0)));
        m_driverController.povUp().whileTrue(new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_REST_ANGLE));
        m_driverController.povDown().whileTrue(new ArmOutCommand(armSubsystem, ArmMotorConstants.ARM_DEPLOY_ANGLE));
        m_driverController.a()
                .whileTrue(new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED));
        m_driverController.povRight().onTrue(new InstantCommand((() -> m_shooter.setHoodPosition(angleSupplier))));
        m_operatorController.leftBumper().whileTrue(
                new AimAtHubCommand(m_turret, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds));
        m_driverController.leftBumper().whileTrue(new InstantCommand(() -> this.changeMaxSpeed(Constants.maxSpeedFast)))
                .onFalse(new InstantCommand(() -> this.changeMaxSpeed(Constants.maxSpeedSlow)));

        m_driverController.rightTrigger(0.5).onTrue(Commands.runOnce(()->{changeMaxSpeed(2, RotationsPerSecond.of(0.5).in(RadiansPerSecond));})).onFalse(Commands.runOnce(()->{changeMaxSpeed(Constants.maxSpeedFast, Constants.MaxAngularSpeedNormal);}));

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
                .onFalse(new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(0)));

        m_driverController.rightBumper()
                .whileTrue(m_Swerve.applyRequest(() -> brake).alongWith(new InstantCommand(() -> this.brakeMode(true))))
                .onFalse(new InstantCommand(() -> this.brakeMode(false)));
        m_operatorController.a().and(m_operatorController.b().negate()).whileTrue(new FeedCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds));
        m_operatorController.a().and(m_operatorController.b()).whileTrue(new FeedCommand2(m_turret, m_shooter, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds));
        

        // m_operatorController.rightTrigger().whileTrue(
        // new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, ()
        // -> m_Swerve.getState().Speeds));

        m_driverController.start()
                .onTrue(new InstantCommand(() -> m_Swerve.resetRotation(
                        DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(180)
                                : Rotation2d.fromDegrees(0))));

        // m_operatorController.povUp().whileTrue(new InstantCommand(() ->
        // armSubsystem.runArm(ArmMotorConstants.ARM_SPEED)));
        // m_operatorController.povDown()
        // .whileTrue(new InstantCommand(() ->
        // armSubsystem.runArm(-ArmMotorConstants.ARM_SPEED)));

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


        // m_operatorController.start().whileTrue(
        //         new InstantCommand(() -> {
        //                 leftRightTrim = m_operatorController.getRightX() * TRIM_DEGREES;
        //                 Logger.recordOutput("LeftRightTrim", leftRightTrim);
        //         })
        // );

        // m_operatorController.back().onTrue(
        //         new InstantCommand(() -> {
        //                 leftRightTrim = 0.0;
        //                 Logger.recordOutput("LeftRightTrim", leftRightTrim);
        //         })
        // );

        m_operatorController.rightTrigger().and(m_operatorController.start()).whileTrue(new ShootAtHubCommand(
                        m_turret,
                        m_shooter,
                        () -> m_Swerve.getState().Pose,
                        () -> m_Swerve.getState().Speeds,
                        () -> m_operatorController.getRightX() * TRIM_DEGREES,
                        () -> 0.0
                ));

        // No trim
        m_operatorController.rightTrigger().and(m_operatorController.start().negate())
                .whileTrue(new ShootAtHubCommand(
                        m_turret,
                        m_shooter,
                        () -> m_Swerve.getState().Pose,
                        () -> m_Swerve.getState().Speeds
                ));

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

    public void changeMaxSpeed(double newMaxSpeed) {
        Logger.recordOutput("Swerve/MaxSpeed", newMaxSpeed);
        MaxSpeed = () -> newMaxSpeed;
    }

    public void changeMaxSpeed(double newMaxSpeed, double newMaxAngularSpeed) {
        Logger.recordOutput("Swerve/MaxSpeed", newMaxSpeed);
        MaxSpeed = () -> newMaxSpeed;
        MaxAngularRate = () -> newMaxAngularSpeed; 
    }

        public void brakeMode(boolean enable) {
        if (enable) {
                Logger.recordOutput("Swerve/BrakeMode", true);
                Logger.recordOutput("Swerve/BrakeModeColor", "#15ff00ff");
        } else {
                Logger.recordOutput("Swerve/BrakeMode", false);
                Logger.recordOutput("Swerve/BrakeModeColor", "#0a00d0ff");
        }
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

