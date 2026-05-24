package frc.robot.OpModes;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootingCommands;
import frc.robot.util.ManualFlywheelSpeed;
import frc.robot.util.Telemetry;

@Teleop(name = "Teleop", group = "Teleop")
public class TeleopOpMode extends CommandOpMode { //type:ignore CommandOpMode isnt implemented by wpilib yet but is planned
        private final Robot robot;
        private final CommandNiDsXboxController m_driverController = new CommandNiDsXboxController(
                        OperatorConstants.kDriverControllerPort);
        private final CommandNiDsXboxController m_operatorController = new CommandNiDsXboxController(
                        OperatorConstants.kOperatorControllerPort);
        private double leftRightTrim = 0.0;
        private static final double TRIM_DEGREES = 10.0d;

        LoggedNetworkNumber speedInput = new LoggedNetworkNumber("Tuning/Speed",0d);
        LoggedNetworkNumber angleInput = new LoggedNetworkNumber("Tuning/Angle",0d);
        DoubleSupplier speedSupplier = () -> speedInput.get();
        DoubleSupplier angleSupplier = () -> angleInput.get();

        private Supplier<Double> MaxSpeed = () -> Constants.maxSpeedSlow;
        private Supplier<Double> MaxAngularRate = () -> RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
                                                                                                          // rotation
                                                                                                          // per
                                                                                                          // second
        // max angular velocity

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed.get() * 0.1).withRotationalDeadband(MaxAngularRate.get() * 0.1) // Add a
                                                                                                               // 10%
                                                                                                               // deadband
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
                                // robot.m_Swerve will execute this Command periodically
                                robot.m_Swerve.applyRequest(() -> drive
                                                .withVelocityX(-m_driverController.getLeftY() * MaxSpeed.get()) // Drive
                                                                                                                // forward
                                                                                                                // with
                                                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed.get()) // Drive
                                                                                                                // left
                                                                                                                // with
                                                                                                                // negative
                                                                                                                // X
                                                                                                                // (left)
                                                .withRotationalRate(
                                                                -m_driverController.getRightX() * MaxAngularRate.get()) // Drive
                                                                                                                        // counterclockwise
                                // with
                                // negative X (left)
                                ));

                robot.m_Swerve.registerTelemetry(new Telemetry(MaxSpeed.get())::telemeterize);

                m_driverController.x().whileTrue(robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED).alongWith(
                        Command.sequence(
                                this.robot.spindexer.runSpindexerCommand(SpindexerConstants.jamRecoverySpeed).withTimeout(Seconds.of(0.25)),
                                this.robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED)).withAutomaticName()).named("Spindexer - Driver X"));
                // m_driverController.y().whileTrue(new InstantCommand(() ->
                // m_kicker.runKicker(KickerConstants.KICKER_SPEED)))
                // .whileFalse(new InstantCommand(() -> m_kicker.runKicker(0)));
                m_driverController.povUp().whileTrue(robot.armSubsystem.runArmCommand(ArmMotorConstants.ARM_REST_ANGLE));
                m_driverController.povDown()
                                .whileTrue(robot.armSubsystem.runArmCommand(ArmMotorConstants.ARM_DEPLOY_ANGLE));
                m_driverController.a()
                                .whileTrue(robot.intakeSubsystem.runIntakeCommand(IntakeConstants.INTAKE_IN_SPEED));
                m_driverController.povRight().onTrue(Command.noRequirements((co -> {robot.m_shooter.setHoodPosition(angleSupplier);})).named("Set Hood Position"));
                // m_operatorController.leftBumper().whileTrue(
                //                 new AimAtHubCommand(m_turret, () -> robot.m_Swerve.getState().Pose,
                //                                 () -> robot.m_Swerve.getState().Speeds));
                m_driverController.leftBumper()
                                .whileTrue(Command.noRequirements(co -> this.changeMaxSpeed(Constants.maxSpeedFast)).named("Max Speed Fast"))
                                .onFalse(Command.noRequirements(co -> this.changeMaxSpeed(Constants.maxSpeedSlow)).named("Max Speed Normal"));

                m_driverController.rightTrigger(0.5).onTrue(Command.noRequirements(_ -> {
                        changeMaxSpeed(2, RotationsPerSecond.of(0.5).in(RadiansPerSecond));
                }).named("Change speed")).onFalse(Command.noRequirements(_ -> {
                        changeMaxSpeed(Constants.maxSpeedFast, Constants.MaxAngularSpeedNormal);
                }).named("Reset Speed"));

                opRJoystickX.and(m_operatorController.start().negate())
                                .whileTrue(robot.m_turret.runTurretCommand(opRightX));

                opLJoystickY.whileTrue(Command.noRequirements(co -> {
                        if (opLeftY.getAsDouble()>0d){
                                co.await(robot.armSubsystem.runArmCommand(
                                        () -> ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE,
                                        () -> Math.abs(opLeftY.getAsDouble())* ArmMotorConstants.ARM_SPEED));
                        }
                        else{
                                co.await(robot.armSubsystem.runArmCommand(
                                        () -> ArmConstants.ArmMotorConstants.ARM_REST_ANGLE,
                                        () -> Math.abs(opLeftY.getAsDouble())* ArmMotorConstants.ARM_SPEED));
                        }
                }).named("Arm - Op Left Y"));
                                

                // m_driverController.povLeft()
                // .whileTrue(Command.parallel(new RunKickerCommand(m_kicker,
                // KickerConstants.KICKER_SPEED),
                // new WaitCommand(0.75).andThen(new RunSpindexerCommand(spindexer,
                // SpindexerConstants.SPINDEXER_SPEED))));

                m_driverController.b()
                                .whileTrue(robot.m_shooter.runFlywheel(() -> RotationsPerSecond.of(speedSupplier.getAsDouble())))
                                .onFalse(robot.m_shooter.runFlywheel(RotationsPerSecond.of(0)));

                m_driverController.rightBumper()
                                .whileTrue(robot.m_Swerve.applyRequest(() -> brake)
                                                .alongWith( Command.noRequirements(co -> this.brakeMode(true)).named("Brake Mode ")))
                                .onFalse(Command.noRequirements(co -> this.brakeMode(false)).named("No Brake Mode "));

                m_operatorController.a().and(m_operatorController.b().negate()).whileTrue(ShootingCommands.feed(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,() -> robot.m_Swerve.getState().Speeds));
                m_operatorController.a().and(m_operatorController.b()).whileTrue(ShootingCommands.feed2(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,() -> robot.m_Swerve.getState().Speeds));

                // m_operatorController.rightTrigger().whileTrue(
                // new ShootAtHubCommand(m_turret, m_shooter, () ->
                // robot.m_Swerve.getState().Pose, ()
                // -> robot.m_Swerve.getState().Speeds));

                m_driverController.start().onTrue(Command.noRequirements(co -> robot.m_Swerve.resetRotation(
                        DriverStation.getAlliance().get() == Alliance.RED
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0))).named("Reset field centric heading"));

                // m_operatorController.povUp().whileTrue(new InstantCommand(() ->
                // armSubsystem.runArm(ArmMotorConstants.ARM_SPEED)));
                // m_operatorController.povDown()
                // .whileTrue(new InstantCommand(() ->
                // armSubsystem.runArm(-ArmMotorConstants.ARM_SPEED)));

                m_operatorController.povUp().onTrue(Command.noRequirements(co -> {ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed1);}).named("Manual Speed 1 (I)"));
                m_operatorController.povRight().onTrue(Command.noRequirements(co -> {ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed2);}).named("Manual Speed 2 (I)"));
                m_operatorController.povLeft().onTrue(Command.noRequirements(co -> {ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed3);}).named("Manual Speed 3 (I)"));
                m_operatorController.povDown().onTrue(Command.noRequirements(co -> {ManualFlywheelSpeed.setSpeed(FlywheelConstants.manualSpeed4);}).named("Manual Speed 4 (I)"));

                m_operatorController.y()
                                .whileTrue(Command.parallel(
                                                Command.waitFor(Seconds.of(0.5)).named("Wait")
                                                                .andThen(robot.m_turret.runTurretToAngleCommand(TurretConstants.manualAngle1)).named("Turn Turret"),
                                                robot.m_shooter.runFlywheel(() -> ManualFlywheelSpeed.getSpeed())).withAutomaticName());
                m_operatorController.x()
                                .whileTrue(Command.parallel(
                                                Command.waitFor(Seconds.of(0.5)).named("Wait").andThen(robot.m_turret.runTurretToAngleCommand(TurretConstants.manualAngle2)).named("Turret to Angle (Operator X)"),
                                                robot.m_shooter.runFlywheel(() -> ManualFlywheelSpeed.getSpeed())).withAutomaticName());
                m_operatorController.b().and(m_operatorController.a().negate())
                                .whileTrue(Command.parallel(
                                                Command.waitFor(Seconds.of(0.5)).named("Wait").andThen(robot.m_turret.runTurretToAngleCommand(TurretConstants.manualAngle3)).named("Turret to Angle (Operator B)"),
                                                robot.m_shooter.runFlywheel(() -> ManualFlywheelSpeed.getSpeed())).withAutomaticName());

                m_operatorController.rightBumper().whileTrue(Command.parallel(robot.m_shooter.runFlywheel(() -> ManualFlywheelSpeed.getSpeed())).withAutomaticName());

                m_operatorController.start().whileTrue(
                                Command.noRequirements(co -> {
                                        leftRightTrim = m_operatorController.getRightX() * TRIM_DEGREES;
                                        Logger.recordOutput("LeftRightTrim", leftRightTrim);
                                }).named("Trim"));

                m_operatorController.back().onTrue(
                                Command.noRequirements(co -> {
                                        leftRightTrim = 0.0;
                                        Logger.recordOutput("LeftRightTrim", leftRightTrim);
                                }).named("Start Trim"));

                m_operatorController.rightTrigger().and(m_operatorController.start()).whileTrue(ShootingCommands.shootAtHub(
                                robot.m_turret,
                                robot.m_shooter,
                                () -> robot.m_Swerve.getState().Pose,
                                () -> robot.m_Swerve.getState().Speeds,
                                () -> m_operatorController.getRightX() * TRIM_DEGREES,
                                () -> 0.0));

                // No trim
                m_operatorController.rightTrigger().and(m_operatorController.start().negate())
                                .whileTrue(ShootingCommands.shootAtHub(
                                                robot.m_turret,
                                                robot.m_shooter,
                                                () -> robot.m_Swerve.getState().Pose,
                                                () -> robot.m_Swerve.getState().Speeds));

                m_operatorController.leftTrigger().whileTrue(robot.intakeSubsystem.runIntakeCommand(IntakeConstants.OUTTAKE_SPEED));
        }

        public Angle nextArmAngle() {
                if (Math.abs(robot.armSubsystem.getArmEncoder().minus(ArmMotorConstants.ARM_DEPLOY_ANGLE)
                                .in(Degrees)) < Math
                                                .abs(robot.armSubsystem.getArmEncoder()
                                                                .minus(ArmMotorConstants.ARM_REST_ANGLE)
                                                                .in(Degrees))) {
                        return ArmMotorConstants.ARM_REST_ANGLE;
                }
                return ArmMotorConstants.ARM_DEPLOY_ANGLE;
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
                if (Math.random() < 0.01) {
                        Runtime.getRuntime().gc();
                }
                Logger.recordOutput("Swerve Current", robot.m_Swerve.getModules()[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
        }
}
