package frc.robot.OpModes;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.ShootingCommands.AimingConstants;

import frc.robot.Robot;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.util.ShotSolution;

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
                new ArmOutCommand(robot.armSubsystem, ArmMotorConstants.ARM_DEPLOY_ANGLE).withTimeout(Seconds.of(20)));
        NamedCommands.registerCommand("ArmIn", new ArmOutCommand(robot.armSubsystem, ArmMotorConstants.ARM_REST_ANGLE));
        NamedCommands.registerCommand("Intake",
                new RunIntakeCommand(robot.intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED).withTimeout(Seconds.of(3)));

        NamedCommands.registerCommand("Spindexer", robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED).withTimeout(Seconds.of(6)));
        NamedCommands.registerCommand("Kicker",robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED).withTimeout(Seconds.of(8)));

        NamedCommands.registerCommand("LongSpindexer",
                robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED).withTimeout(Seconds.of(11)));
        NamedCommands.registerCommand("LongKicker",
                robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED).withTimeout(Seconds.of(11)));

        NamedCommands.registerCommand("LongIntake",
                new RunIntakeCommand(robot.intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("LongishIntake",
                new RunIntakeCommand(robot.intakeSubsystem, ledSubsystem, IntakeConstants.INTAKE_IN_SPEED).withTimeout(Seconds.of(3.7)));

        NamedCommands.registerCommand("ShootFromLeftBump",
                new TakeShotCommand(robot.m_turret, robot.m_shooter, TakeShotCommand.ShotData.leftBump).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("ShootFromLeftTrench",
                new TakeShotCommand(robot.m_turret, robot.m_shooter, TakeShotCommand.ShotData.leftTrench).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongShootFromLeftTrench",
                new TakeShotCommand(robot.m_turret, robot.m_shooter, TakeShotCommand.ShotData.leftTrench).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("ShootFromRightTrench",
                new TakeShotCommand(robot.m_turret, robot.m_shooter, TakeShotCommand.ShotData.rightTrench).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongShootFromRightTrench",
                new TakeShotCommand(robot.m_turret, robot.m_shooter, TakeShotCommand.ShotData.rightTrench).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("AimFromTrench",
                new AimAtHubCommand(robot.m_turret, () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds)
                        .withTimeout(Seconds.of(0.2)));

        NamedCommands.registerCommand("LeftFlywheelFromTrench",
                robot.m_shooter.runFlywheel(RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed))
                        .withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongLeftFlywheelFromTrench",
                robot.m_shooter.runFlywheel(RotationsPerSecond.of(FlywheelConstants.leftTrenchSpeed))
                        .withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("RightFlywheelFromTrench",
                robot.m_shooter.runFlywheel(RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed))
                        .withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongRightFlywheelFromTrench",
                robot.m_shooter.runFlywheel(RotationsPerSecond.of(FlywheelConstants.rightTrenchSpeed))
                        .withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("FullVision",
                ShootingCommands.shootAtHub(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,
                        () -> robot.m_Swerve.getState().Speeds).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongFullVision",
                ShootingCommands.shootAtHub(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,
                        () -> robot.m_Swerve.getState().Speeds).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("FeedFromNeutral",
                ShootingCommands.feed(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds)
                        .withTimeout(Seconds.of(6)));

        NamedCommands.registerCommand("CenterShoot", new VisionFixedSpeedCommand(robot.m_turret, robot.m_shooter,
                new ShotSolution(-1, 52, -1), () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds));
        NamedCommands.registerCommand("Shoot3sec",
                Command.parallel(
                        ShootingCommands.shootAtHub(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,
                                () -> robot.m_Swerve.getState().Speeds),
                        new ConditionalCommand(robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED).andThen(
                                new WaitCommand(0.75).andThen(
                                        new RunSpindexerCommand(robot.spindexer, SpindexerConstants.SPINDEXER_SPEED))),
                                new InstantCommand(), flywheelAtSpeed))
                        .withTimeout(Seconds.of(3)));

        NamedCommands.registerCommand("Shoot6sec",
                Command.parallel(
                        ShootingCommands.shootAtHub(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,
                                () -> robot.m_Swerve.getState().Speeds),
                        robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED)
                                .alongWith(robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED)))
                        .withTimeout(Seconds.of(6)));

    }

}
