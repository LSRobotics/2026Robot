package frc.robot.OpModes;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Robot;

@Autonomous(name = "Pathplanner Auto", group = "Auto")
public class PathplannerAuto extends PeriodicOpMode {
    private final Robot robot;
    private final SendableChooser<Command> autoChooser;

    public PathplannerAuto(Robot robot) {
        this.robot = robot;
        configureNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    @Override
    public void start() {
        Scheduler.getDefault().schedule(getAutonomousCommand());
    }

    @Override
    public void end() {
        Scheduler.getDefault().cancel(getAutonomousCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureNamedCommands() {
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
                new AimAtHubCommand(m_turret, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds)
                        .withTimeout(0.2));

        NamedCommands.registerCommand("LeftFlywheelFromTrench",
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed))
                        .withTimeout(4));

        NamedCommands.registerCommand("LongLeftFlywheelFromTrench",
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed))
                        .withTimeout(10));

        NamedCommands.registerCommand("RightFlywheelFromTrench",
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed))
                        .withTimeout(4));

        NamedCommands.registerCommand("LongRightFlywheelFromTrench",
                new RunFlywheelCommand(m_shooter, RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed))
                        .withTimeout(10));

        NamedCommands.registerCommand("FullVision",
                new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose,
                        () -> m_Swerve.getState().Speeds).withTimeout(4));

        NamedCommands.registerCommand("LongFullVision",
                new ShootAtHubCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose,
                        () -> m_Swerve.getState().Speeds).withTimeout(10));

        NamedCommands.registerCommand("FeedFromNeutral",
                new FeedCommand(m_turret, m_shooter, () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds)
                        .withTimeout(6));

        NamedCommands.registerCommand("CenterShoot", new VisionFixedSpeedCommand(m_turret, m_shooter,
                new ShotSolution(-1, 52, -1), () -> m_Swerve.getState().Pose, () -> m_Swerve.getState().Speeds));
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

    }

}
