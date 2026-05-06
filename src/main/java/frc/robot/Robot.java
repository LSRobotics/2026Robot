// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import static org.wpilib.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.BuildConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.Shooter.ShooterFlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.ShooterHoodIOLinearActuator;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOTalon;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmLimitSwitchIOLimitSwitch;
import frc.robot.subsystems.arm.ArmMechanism;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeMechanism;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.subsystems.leds.LEDMechanism;
import frc.robot.subsystems.leds.LedsIOBlinkin;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.util.GameTimers;
import frc.robot.util.PhaseTimer;
import frc.robot.util.ShotSolution;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends OpModeRobot {
  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;

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

    private final IntakeIOTalonFX intakeIO = new IntakeIOTalonFX();
    private final IntakeMechanism intakeSubsystem = new IntakeMechanism(intakeIO);

    private final ArmIOSparkMax armIO = new ArmIOSparkMax();
    private final ArmLimitSwitchIOLimitSwitch limitSwitchIO = new ArmLimitSwitchIOLimitSwitch();
    private final ArmMechanism armSubsystem = new ArmMechanism(armIO, limitSwitchIO);

    private final LedsIOBlinkin ledIO = new LedsIOBlinkin();
    private final LEDMechanism ledSubsystem = new LEDMechanism(ledIO);

    private final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();

    private final VisionSubsystem m_Vision = new VisionSubsystem(m_Swerve::addVisionMeasurement,
            new VisionIOPhotonVision(VisionConstants.camera0name, VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(VisionConstants.camera1name, VisionConstants.robotToCamera1),
            new VisionIOPhotonVision(VisionConstants.camera2name, VisionConstants.robotToCamera2));

    private Trigger flywheelAtSpeed = new Trigger(
            () -> (m_shooter.getFlywheelVelocity().minus(m_shooter.targetSpeed)
                    .abs(RotationsPerSecond)) <= (ShooterConstants.FlywheelConstants.flywheelTolerance
                            .in(RotationsPerSecond)));

  
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    LEDManager.init(ledSubsystem);
    LEDManager.setDefault();

    Logger.addDataReceiver(new WPILOGWriter()); // Log to usb (crashes id not
    // present)
    Logger.addDataReceiver(new NT4Publisher());

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    Scheduler.getDefault().run();;
    m_robotContainer.periodic();
    GameTimers.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public void configureNamedCommands() {
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

  }

}
