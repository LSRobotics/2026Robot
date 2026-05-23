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
                robot.armSubsystem.runArmCommand(ArmMotorConstants.ARM_DEPLOY_ANGLE).withTimeout(Seconds.of(20)));
        NamedCommands.registerCommand("ArmIn", robot.armSubsystem.runArmCommand(ArmMotorConstants.ARM_REST_ANGLE));
        NamedCommands.registerCommand("Intake",
                robot.intakeSubsystem.runIntakeCommand(IntakeConstants.INTAKE_IN_SPEED).withTimeout(Seconds.of(3)));

        NamedCommands.registerCommand("Spindexer", robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED).withTimeout(Seconds.of(6)));
        NamedCommands.registerCommand("Kicker",robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED).withTimeout(Seconds.of(8)));

        NamedCommands.registerCommand("LongSpindexer",
                robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED).withTimeout(Seconds.of(11)));
        NamedCommands.registerCommand("LongKicker",
                robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED).withTimeout(Seconds.of(11)));

        NamedCommands.registerCommand("LongIntake",
                robot.intakeSubsystem.runIntakeCommand(IntakeConstants.INTAKE_IN_SPEED).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("LongishIntake",
                robot.intakeSubsystem.runIntakeCommand(IntakeConstants.INTAKE_IN_SPEED).withTimeout(Seconds.of(3.7)));

        NamedCommands.registerCommand("ShootFromLeftBump",
                ShootingCommands.TakeShotCommand(robot.m_turret, robot.m_shooter, ShootingCommands.ShotData.leftBump).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("ShootFromLeftTrench",
                ShootingCommands.TakeShotCommand(robot.m_turret, robot.m_shooter, ShootingCommands.ShotData.leftTrench).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongShootFromLeftTrench",
                ShootingCommands.TakeShotCommand(robot.m_turret, robot.m_shooter, ShootingCommands.ShotData.leftTrench).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("ShootFromRightTrench",
                ShootingCommands.TakeShotCommand(robot.m_turret, robot.m_shooter, ShootingCommands.ShotData.rightTrench).withTimeout(Seconds.of(4)));

        NamedCommands.registerCommand("LongShootFromRightTrench",
                ShootingCommands.TakeShotCommand(robot.m_turret, robot.m_shooter, ShootingCommands.ShotData.rightTrench).withTimeout(Seconds.of(10)));

        NamedCommands.registerCommand("AimFromTrench",
                ShootingCommands.AimAtHubCommand(robot.m_turret, () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds)
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

        NamedCommands.registerCommand("CenterShoot", ShootingCommands.VisionFixedSpeedCommand(robot.m_turret, robot.m_shooter,
                new ShotSolution(-1, 52, -1), () -> robot.m_Swerve.getState().Pose, () -> robot.m_Swerve.getState().Speeds));

        NamedCommands.registerCommand("Shoot3sec", Command.requiring(robot.m_turret, robot.m_shooter).executing(co -> {
            co.fork(ShootingCommands.shootAtHub(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,
                    () -> robot.m_Swerve.getState().Speeds));
                        co.waitUntil(() -> Math.abs(robot.m_shooter.getFlywheelVelocity().in(RotationsPerSecond) 
                    - robot.m_shooter.targetSpeed.in(RotationsPerSecond)) 
                    <= FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));

            co.fork(robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED));
            co.waitFor(Seconds.of(0.75)); //type:ignore Not implemented by wpilib yet but planned
            co.fork(robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED));

            co.park();
        }).named("Shoot3sec").withTimeout(Seconds.of(3)));

        NamedCommands.registerCommand("Shoot6sec", Command.requiring(robot.m_turret, robot.m_shooter).executing(co -> {
            co.fork(ShootingCommands.shootAtHub(robot.m_turret, robot.m_shooter, () -> robot.m_Swerve.getState().Pose,
                    () -> robot.m_Swerve.getState().Speeds));
            co.fork(robot.spindexer.runSpindexerCommand(SpindexerConstants.SPINDEXER_SPEED));
            co.fork(robot.m_kicker.runKickerCommand(KickerConstants.KICKER_SPEED));
            co.park();
        }).named("Shoot6sec").withTimeout(Seconds.of(6)));

    }

}
