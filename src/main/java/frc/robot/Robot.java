// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent.Completed;
import org.wpilib.command3.SchedulerEvent.CompletedWithError;
import org.wpilib.command3.SchedulerEvent.Interrupted;
import org.wpilib.command3.SchedulerEvent.*;
import org.wpilib.command3.SchedulerEvent.Mounted;
import org.wpilib.command3.SchedulerEvent.Scheduled;
import org.wpilib.command3.SchedulerEvent.Yielded;
import org.wpilib.command3.SchedulerEvent.Canceled;
import org.wpilib.command3.SchedulerEvent.Interrupted;
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
import frc.robot.subsystems.Shooter.ShooterMechanism;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOTalon;
import frc.robot.subsystems.Turret.TurretMechanism;
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
import frc.robot.subsystems.kicker.KickerMechanism;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.subsystems.leds.LEDMechanism;
import frc.robot.subsystems.leds.LedsIOBlinkin;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.spindexer.SpindexerMechanism;
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
  public final SpindexerMechanism spindexer = new SpindexerMechanism(SpinIO);
  private final KickerIO kickerIO = new KickerIOTalonFX();
  public final KickerMechanism m_kicker = new KickerMechanism(kickerIO);
  private final TurretIO turretIO = new TurretIOTalon();
  public final TurretMechanism m_turret = new TurretMechanism(turretIO);

  private final SendableChooser<Command> autoChooser;

  public final ShooterMechanism m_shooter = new ShooterMechanism(new ShooterFlywheelIOTalonFX(),
      new ShooterHoodIOLinearActuator(ShooterConstants.HoodConstants.hoodLinearActuatorPWMID,
          ShooterConstants.HoodConstants.hoodLinearActuatorPWMID2));

  // Replace with CommandPS4Controller or Commandm_driverController if needed

  private final IntakeIOTalonFX intakeIO = new IntakeIOTalonFX();
  public final IntakeMechanism intakeSubsystem = new IntakeMechanism(intakeIO);

  private final ArmIOSparkMax armIO = new ArmIOSparkMax();
  private final ArmLimitSwitchIOLimitSwitch limitSwitchIO = new ArmLimitSwitchIOLimitSwitch();
  public final ArmMechanism armSubsystem = new ArmMechanism(armIO, limitSwitchIO);

  private final LedsIOBlinkin ledIO = new LedsIOBlinkin();
  private final LEDMechanism ledSubsystem = new LEDMechanism(ledIO);

  public final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();

  private final VisionSubsystem m_Vision = new VisionSubsystem(m_Swerve::addVisionMeasurement,
      new VisionIOPhotonVision(VisionConstants.camera0name, VisionConstants.robotToCamera0),
      new VisionIOPhotonVision(VisionConstants.camera1name, VisionConstants.robotToCamera1),
      new VisionIOPhotonVision(VisionConstants.camera2name, VisionConstants.robotToCamera2));

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

    /*
     * Event Description
     * Scheduled(Command, Time) A command was queued to run
     * Mounted(Command, Time) The scheduler started or resumed a command
     * Yielded(Command, Time) A command paused with Coroutine.yield()
     * Completed(Command, Time) A command successfully completed
     * CompletedWithError(Command, Throwable, Time) A command encountered an
     * unhandled exception
     * Canceled(Command, Time) A command was canceled (various causes)
     * Interrupted(Command, Command, Time) A command was interrupted by another
     */

    Scheduler.getDefault().addEventListener(event -> {
      switch (event) {
        case Scheduled(var cmd, var time) ->
          System.out.println("Scheduled " + cmd.name() + " at " + time);
        case Mounted(var cmd, var time) ->
          System.out.println("Resumed " + cmd.name() + " at " + time);
        case Yielded(var cmd, var time) ->
          System.out.println("Paused " + cmd.name() + " at " + time);
        case Completed(var cmd, var time) ->
          System.out.println("Finished " + cmd.name() + " at " + time);
        case CompletedWithError(var cmd, var error, var time) ->
          System.out.println("Errored " + cmd.name() + " at " + time + " with " + error);
        case Canceled(var cmd, var time) ->
          System.out.println("Canceled " + cmd.name() + " at " + time);
        case Interrupted(var cmd, var by, var time) ->
          System.out.println("Interrupted " + cmd.name() + " by " + by.name() + " at " + time);
        default -> {
        }
      }
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
    Scheduler.getDefault().run();
    ;
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

}
