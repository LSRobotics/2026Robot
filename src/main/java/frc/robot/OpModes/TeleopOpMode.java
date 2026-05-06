package frc.robot.OpModes;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.ManualFlywheelSpeed;

@Teleop(name = "Teleop", group = "Teleop")
public class TeleopOpMode extends PeriodicOpMode {
    private final Robot robot;
    private final CommandNiDsXboxController m_driverController = new CommandNiDsXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandNiDsXboxController m_operatorController = new CommandNiDsXboxController(
            OperatorConstants.kOperatorControllerPort);
    private double leftRightTrim = 0.0;
    private static final double TRIM_DEGREES = 10.0d;

    private Supplier<Double> MaxSpeed = () -> Constants.maxSpeedSlow;
    private Supplier<Double> MaxAngularRate = () -> RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
                                                                                                      // rotation per
                                                                                                      // second
    // max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.get() * 0.1).withRotationalDeadband(MaxAngularRate.get() * 0.1) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    DoubleSupplier opRightX = () -> m_operatorController.getRightX();
    DoubleSupplier opLeftY = () -> m_operatorController.getLeftY();
    Trigger opRJoystickX = new Trigger(() -> opRightX.getAsDouble() != 0);
    Trigger opLJoystickY = new Trigger(() -> opLeftY.getAsDouble() != 0);

    private Trigger flywheelAtSpeed = new Trigger(
            () -> ((robot.m_shooter.getFlywheelVelocity().minus(robot.m_shooter.targetSpeed)
                    .abs(RotationsPerSecond)) <= (ShooterConstants.FlywheelConstants.flywheelTolerance
                            .in(RotationsPerSecond))));;

    public TeleopOpMode(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        Logger.recordOutput("Swerve/BrakeMode", false);
        Logger.recordOutput("Swerve/BrakeModeColor", "#000000");
        Logger.recordOutput("Swerve/MaxSpeed", MaxSpeed.get());
        ManualFlywheelSpeed.init();

        configureBindings();
    }

    public void configureBindings() {
        robot.m_Swerve.setDefaultCommand(
                // robot.m_Swerve will execute this command periodically
                robot.m_Swerve.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed.get()) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed.get()) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate.get()) // Drive
                                                                                                    // counterclockwise
                // with
                // negative X (left)
                ));

        robot.m_Swerve.registerTelemetry(new Telemetry(MaxSpeed.get())::telemeterize);

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
                new AimAtHubCommand(m_turret, () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds));
        m_driverController.leftBumper().whileTrue(new InstantCommand(() -> this.changeMaxSpeed(Constants.maxSpeedFast)))
                .onFalse(new InstantCommand(() -> this.changeMaxSpeed(Constants.maxSpeedSlow)));

        m_driverController.rightTrigger(0.5).onTrue(Commands.runOnce(() -> {
            changeMaxSpeed(2, RotationsPerSecond.of(0.5).in(RadiansPerSecond));
        })).onFalse(Commands.runOnce(() -> {
            changeMaxSpeed(Constants.maxSpeedFast, Constants.MaxAngularSpeedNormal);
        }));

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
                .whileTrue(new RunFlywheelCommand(robot.m_shooter, () -> RotationsPerSecond.of(speedSupplier.getAsDouble())))
                .onFalse(new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(0)));

        m_driverController.rightBumper()
                .whileTrue(robot.m_Swerve.applyRequest(() -> brake).alongWith(new InstantCommand(() -> this.brakeMode(true))))
                .onFalse(new InstantCommand(() -> this.brakeMode(false)));
        m_operatorController.a().and(m_operatorController.b().negate()).whileTrue(
                new FeedCommand(m_turret, m_shooter, () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds));
        m_operatorController.a().and(m_operatorController.b()).whileTrue(new FeedCommand2(m_turret, m_shooter,
                () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds));

        // m_operatorController.rightTrigger().whileTrue(
        // new ShootAtHubCommand(m_turret, m_shooter, () -> robot.m_Swerve.getState().Pose, ()
        // -> robot.m_Swerve.getState().Speeds));

        m_driverController.start()
                .onTrue(new InstantCommand(() -> robot.m_Swerve.resetRotation(
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
        // new InstantCommand(() -> {
        // leftRightTrim = m_operatorController.getRightX() * TRIM_DEGREES;
        // Logger.recordOutput("LeftRightTrim", leftRightTrim);
        // })
        // );

        // m_operatorController.back().onTrue(
        // new InstantCommand(() -> {
        // leftRightTrim = 0.0;
        // Logger.recordOutput("LeftRightTrim", leftRightTrim);
        // })
        // );

        m_operatorController.rightTrigger().and(m_operatorController.start()).whileTrue(new ShootAtHubCommand(
                m_turret,
                m_shooter,
                () -> robot.m_Swerve.getState().Pose,
                () -> robot.m_Swerve.getState().Speeds,
                () -> m_operatorController.getRightX() * TRIM_DEGREES,
                () -> 0.0));

        // No trim
        m_operatorController.rightTrigger().and(m_operatorController.start().negate())
                .whileTrue(new ShootAtHubCommand(
                        m_turret,
                        m_shooter,
                        () -> robot.m_Swerve.getState().Pose,
                        () -> robot.m_Swerve.getState().Speeds));

        m_operatorController.leftTrigger()
                .whileTrue(new RunIntakeCommand(intakeSubsystem, ledSubsystem, IntakeConstants.OUTTAKE_SPEED));
    }
}
