package frc.robot.commands.systemtest;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ArmOutCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class SystemTest {

    public static final double TEST_INTAKE_MIN_RPS = 5.0;
    public static final double TEST_SPINDEXER_MIN_RPS = 0.2;
    public static final double TEST_KICKER_MIN_RPS = 5.0;
    public static final double TEST_FLYWHEEL_VOLTS = 6.0;
    public static final double TEST_FLYWHEEL_SPINUP_SECONDS = 2.0;
    public static final double TEST_FLYWHEEL_MIN_RPS = 10.0;
    public static final double TEST_TURRET_SPEED = TurretConstants.maxControlSpeed * 0.5;
    public static final double TEST_TURRET_SECONDS = 0.75;
    public static final double TEST_TURRET_MIN_MOVE_DEG = 3.0;
    public static final double TEST_ARM_TIMEOUT = 8.0;
    public static final double TEST_SPINDEXER_SECONDS = 4.0;

    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;
    private final KickerSubsystem kicker;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final ArmSubsystem arm;

    private final List<TestStage> stages = new ArrayList<>();
    private final SendableChooser<String> stageChooser = new SendableChooser<>();

    final AtomicBoolean confirmPressed = new AtomicBoolean(false);
    final AtomicBoolean restartPressed = new AtomicBoolean(false);
    final AtomicBoolean jumpPressed = new AtomicBoolean(false);

    public SystemTest(IntakeSubsystem intake, SpindexerSubsystem spindexer, KickerSubsystem kicker,
            ShooterSubsystem shooter, TurretSubsystem turret, ArmSubsystem arm) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.kicker = kicker;
        this.shooter = shooter;
        this.turret = turret;
        this.arm = arm;

        buildStages();

        stageChooser.setDefaultOption("First Stage", "");
        for (TestStage s : stages) {
            stageChooser.addOption(s.name, s.name);
            SmartDashboard.putString(key(s), "PENDING");
        }
        SmartDashboard.putData("SystemTest/Stage Select", stageChooser);
        SmartDashboard.putData("SystemTest/Confirm Next",
                new InstantCommand(() -> confirmPressed.set(true)));
        SmartDashboard.putData("SystemTest/Restart Stage",
                new InstantCommand(() -> restartPressed.set(true)));
        SmartDashboard.putData("SystemTest/Jump To Selected",
                new InstantCommand(() -> jumpPressed.set(true)));
        SmartDashboard.putData("SystemTest/Run Full Test", fullTest());
        SmartDashboard.putString("SystemTest/Prompt", "Idle");
        SmartDashboard.putString("SystemTest/Current Stage", "None");
    }

    private void buildStages() {
        {
            final double[] peak = new double[1];
            stages.add(stage("01 Intake Forward",
                    () -> peak[0] = 0,
                    Commands.run(() -> intake.runIntake(IntakeConstants.INTAKE_IN_SPEED), intake)
                            .withTimeout(1.5),
                    () -> peak[0] = Math.max(peak[0],
                            Math.abs(intake.getRollerSpeed().in(RotationsPerSecond))),
                    () -> speedCheck("intake roller", peak[0], TEST_INTAKE_MIN_RPS),
                    () -> intake.runIntake(0)));
        }

        {
            final double[] peak = new double[1];
            stages.add(stage("02 Intake Reverse",
                    () -> peak[0] = 0,
                    Commands.run(() -> intake.runIntake(IntakeConstants.OUTTAKE_SPEED), intake)
                            .withTimeout(1.5),
                    () -> peak[0] = Math.max(peak[0],
                            Math.abs(intake.getRollerSpeed().in(RotationsPerSecond))),
                    () -> speedCheck("intake roller (reverse)", peak[0], TEST_INTAKE_MIN_RPS),
                    () -> intake.runIntake(0)));
        }

        {
            final double[] peak = new double[1];
            stages.add(stage("03 Spindexer Forward",
                    () -> peak[0] = 0,
                    Commands.run(() -> spindexer.runSpindexer(SpindexerConstants.SPINDEXER_SPEED), spindexer)
                            .withTimeout(TEST_SPINDEXER_SECONDS),
                    () -> peak[0] = Math.max(peak[0],
                            Math.abs(spindexer.getSpindexerSpeed().in(RotationsPerSecond))),
                    () -> speedCheck("spindexer", peak[0], TEST_SPINDEXER_MIN_RPS),
                    () -> spindexer.runSpindexer(0)));
        }

        {
            final double[] peak = new double[1];
            stages.add(stage("04 Spindexer Reverse",
                    () -> peak[0] = 0,
                    Commands.run(() -> spindexer.runSpindexer(-SpindexerConstants.SPINDEXER_SPEED), spindexer)
                            .withTimeout(TEST_SPINDEXER_SECONDS),
                    () -> peak[0] = Math.max(peak[0],
                            Math.abs(spindexer.getSpindexerSpeed().in(RotationsPerSecond))),
                    () -> speedCheck("spindexer (reverse)", peak[0], TEST_SPINDEXER_MIN_RPS),
                    () -> spindexer.runSpindexer(0)));
        }

        {
            final double[] peak = new double[1];
            stages.add(stage("05 Kicker",
                    () -> peak[0] = 0,
                    Commands.run(() -> kicker.runKicker(KickerConstants.KICKER_SPEED), kicker)
                            .withTimeout(1.5),
                    () -> peak[0] = Math.max(peak[0],
                            Math.abs(kicker.getKickerVelocity().in(RotationsPerSecond))),
                    () -> speedCheck("kicker", peak[0], TEST_KICKER_MIN_RPS),
                    () -> kicker.runKicker(0)));
        }

        {
            final double[] peak = new double[1];
            stages.add(stage("06 Flywheel Spin Up (" + TEST_FLYWHEEL_VOLTS + "V)",
                    () -> peak[0] = 0,
                    Commands.run(() -> shooter.setFlywheelVoltage(Volt.of(TEST_FLYWHEEL_VOLTS)), shooter)
                            .withTimeout(TEST_FLYWHEEL_SPINUP_SECONDS),
                    () -> peak[0] = Math.max(peak[0],
                            Math.abs(shooter.getFlywheelVelocity().in(RotationsPerSecond))),
                    () -> speedCheck("flywheel", peak[0], TEST_FLYWHEEL_MIN_RPS),
                    () -> shooter.setFlywheelVoltage(Volt.of(0))));
        }

        {
            final double[] start = new double[1];
            stages.add(stage("07 Turret Left",
                    () -> start[0] = turret.getAngle().in(Degrees),
                    Commands.run(() -> turret.setSpeed(-TEST_TURRET_SPEED), turret)
                            .withTimeout(TEST_TURRET_SECONDS),
                    null,
                    () -> moveCheck("turret left", turret.getAngle().in(Degrees) - start[0], -1),
                    () -> turret.setSpeed(0)));
        }

        {
            final double[] start = new double[1];
            stages.add(stage("08 Turret Right",
                    () -> start[0] = turret.getAngle().in(Degrees),
                    Commands.run(() -> turret.setSpeed(TEST_TURRET_SPEED), turret)
                            .withTimeout(TEST_TURRET_SECONDS),
                    null,
                    () -> moveCheck("turret right", turret.getAngle().in(Degrees) - start[0], 1),
                    () -> turret.setSpeed(0)));
        }

        stages.add(stage("09 Hood Sweep",
                () -> {},
                Commands.sequence(
                        Commands.runOnce(() -> shooter.setHoodPosition(0.5)),
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> shooter.setHoodPosition(-1.0)),
                        Commands.waitSeconds(1.0)),
                null,
                () -> StageResult.pass("Actuated to both ends of range"),
                () -> {}));

        stages.add(stage("10 Arm Deploy",
                () -> {},
                new ArmOutCommand(arm, ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE)
                        .withTimeout(TEST_ARM_TIMEOUT),
                null,
                () -> armCheck("deploy",
                        ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE.in(Degrees)),
                () -> arm.runArm(0)));

        stages.add(stage("11 Arm Rest",
                () -> {},
                new ArmOutCommand(arm, ArmConstants.ArmMotorConstants.ARM_REST_ANGLE)
                        .withTimeout(TEST_ARM_TIMEOUT),
                null,
                () -> {
                    StageResult r = armCheck("rest",
                            ArmConstants.ArmMotorConstants.ARM_REST_ANGLE.in(Degrees));
                    return StageResult.pass("Arm test complete: " + r.message());
                },
                () -> arm.runArm(0)));
    }

    private TestStage stage(String name, Runnable reset, Command action, Runnable monitor,
            Supplier<StageResult> check, Runnable stop) {
        return new TestStage(name,
                Commands.sequence(Commands.runOnce(reset), action),
                monitor, check, stop);
    }

    private StageResult speedCheck(String label, double measured, double min) {
        if (measured >= min) {
            return StageResult.pass(label + " peaked at " + fmt(measured) + " rps");
        }
        return StageResult.fail(label + " only peaked at " + fmt(measured)
                + " rps (min " + fmt(min) + ") - check for stall/wiring");
    }

    private StageResult moveCheck(String label, double deltaDeg, int expectedSign) {
        boolean movedRightWay = Math.signum(deltaDeg) == expectedSign
                && Math.abs(deltaDeg) >= TEST_TURRET_MIN_MOVE_DEG;
        if (movedRightWay) {
            return StageResult.pass(label + " moved " + fmt(deltaDeg) + " deg");
        }
        return StageResult.fail(label + " moved " + fmt(deltaDeg)
                + " deg (expected >= " + TEST_TURRET_MIN_MOVE_DEG
                + " in direction " + expectedSign + ")");
    }

    private StageResult armCheck(String label, double targetDeg) {
        double current = arm.getArmEncoder().in(Degrees)
                / ArmConstants.ArmMotorConstants.gearRatio;
        double error = Math.abs(current - targetDeg);
        if (error <= ArmConstants.ArmMotorConstants.ARM_TOLERANCE) {
            return StageResult.pass("arm reached " + label + " (" + fmt(current) + " deg)");
        }
        return StageResult.fail("arm at " + fmt(current) + " deg, target "
                + fmt(targetDeg) + " (error " + fmt(error) + ")");
    }

    private static String fmt(double d) {
        return String.format("%.2f", d);
    }

    private static String key(TestStage s) {
        return "SystemTest/" + s.name;
    }

    public Command fullTest() {
        return new Command() {
            private int index;
            private boolean awaitingConfirm;
            private boolean finishedAll;
            private final boolean[] passed = new boolean[stages.size()];

            @Override
            public void initialize() {
                String selection = stageChooser.getSelected();
                index = 0;
                for (int i = 0; i < stages.size(); i++) {
                    passed[i] = false;
                    SmartDashboard.putString(key(stages.get(i)), "PENDING");
                    if (stages.get(i).name.equals(selection)) {
                        index = i;
                    }
                }
                finishedAll = false;
                runStage();
            }

            @Override
            public void execute() {
                if (!awaitingConfirm) {
                    TestStage s = stages.get(index);
                    if (s.monitor != null) {
                        s.monitor.run();
                    }
                    if (!s.action.isScheduled()) {
                        finishStage(s);
                    }
                } else {
                    if (confirmPressed.getAndSet(false)) {
                        if (index == stages.size() - 1) {
                            finishedAll = true;
                        } else {
                            index++;
                            runStage();
                        }
                    } else if (restartPressed.getAndSet(false)) {
                        runStage();
                    } else if (jumpPressed.getAndSet(false)) {
                        String selection = stageChooser.getSelected();
                        for (int i = 0; i < stages.size(); i++) {
                            if (stages.get(i).name.equals(selection)) {
                                index = i;
                            }
                        }
                        runStage();
                    }
                }
            }

            private void runStage() {
                TestStage s = stages.get(index);
                awaitingConfirm = false;
                SmartDashboard.putString("SystemTest/Current Stage", s.name);
                SmartDashboard.putString("SystemTest/Prompt",
                        "RUNNING - listen and watch, then confirm");
                SmartDashboard.putString(key(s), "RUNNING");
                LEDManager.setColor(LEDConstants.colorYellow);
                Logger.recordOutput("SystemTest/CurrentStage", s.name);
                CommandScheduler.getInstance().schedule(s.action);
            }

            private void finishStage(TestStage s) {
                StageResult result = s.check.get();
                passed[index] = result.passed();
                SmartDashboard.putString(key(s),
                        (result.passed() ? "PASS: " : "FAIL: ") + result.message());
                Logger.recordOutput("SystemTest/" + s.name, result.message());
                LEDManager.setColor(result.passed()
                        ? LEDConstants.colorLawnGreen
                        : LEDConstants.colorRed);
                awaitingConfirm = true;
                String next = index == stages.size() - 1
                        ? "end of test"
                        : "next: " + stages.get(index + 1).name;
                SmartDashboard.putNumber("SystemTest/Progress", index + 1);
                SmartDashboard.putNumber("SystemTest/Total Stages", stages.size());
                SmartDashboard.putString("SystemTest/Prompt",
                        (result.passed() ? "PASSED" : "FAILED")
                                + " - Confirm Next for " + next
                                + ", Restart Stage to retry, Jump To Selected to skip around");
            }

            @Override
            public boolean isFinished() {
                return finishedAll;
            }

            @Override
            public void end(boolean interrupted) {
                for (TestStage s : stages) {
                    s.stop.run();
                }
                LEDManager.setDefault();
                int passCount = 0;
                for (boolean p : passed) {
                    if (p) passCount++;
                }
                SmartDashboard.putString("SystemTest/Prompt",
                        interrupted ? "CANCELLED" : "COMPLETE: " + passCount + "/" + stages.size());
                SmartDashboard.putString("SystemTest/Current Stage", "Done");
            }
        }.withName("Full System Test");
    }
}

