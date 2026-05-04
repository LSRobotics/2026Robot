package frc.robot.commands;

import static org.wpilib.units.Units.Degree;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volt;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.DriverStation.Alliance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.util.FieldConstants;
import frc.robot.util.MathUtils;
//Opposing ALliance
public class FeedCommand2 extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private final ShooterSubsystem m_shooter;
    private Pose2d target;
    private PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    private BangBangController flywheelController = new BangBangController();
    // @SuppressWarnings("unchecked")
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.FlywheelConstants.kS.in(Volt),
            ShooterConstants.FlywheelConstants.kV,
            ShooterConstants.FlywheelConstants.kA);
    private  double LEDColor = LEDConstants.defaultColor;

    public FeedCommand2(TurretSubsystem turret, ShooterSubsystem shooter, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turret = turret;
        this.m_shooter = shooter;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        pid.setTolerance(TurretConstants.tolerance);
    }

    @Override
    public void execute() {
        Pose2d target1 = new Pose2d(TurretConstants.blue1, new Rotation2d());
        Pose2d target2 = new Pose2d(TurretConstants.blue2, new Rotation2d());

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) { 
            target1 = new Pose2d(
                    FieldConstants.fieldLength - target1.getX(), 
                    target1.getY(), 
                    target1.getRotation().plus(Rotation2d.fromDegrees(180))
            );

            target2 = new Pose2d(
                FieldConstants.fieldLength - target2.getX(),
                target2.getY(),
                target2.getRotation().plus(Rotation2d.fromDegrees(180)));
        }
        
        target = target1.getTranslation().getDistance(robotPoseSupplier.get().getTranslation())<target2.getTranslation().getDistance(robotPoseSupplier.get().getTranslation()) ? target1 : target2;

        
        Pose2d turretPose = robotPoseSupplier.get().transformBy(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        ChassisSpeeds robotVelocity = chassisSpeedSupplier.get();
        Pose2d predictedTurretPose = turretPose.plus(
                new Transform2d(
                        new Translation2d(
                                robotVelocity.vxMetersPerSecond * TurretConstants.lookaheadLatency.in(Seconds),
                                robotVelocity.vyMetersPerSecond * TurretConstants.lookaheadLatency.in(Seconds)),
                        new Rotation2d(
                                robotVelocity.omegaRadiansPerSecond * TurretConstants.lookaheadLatency.in(Seconds))));
        Rotation2d angleToHub = target.getTranslation().minus(predictedTurretPose.getTranslation()).getAngle();
        angleToHub = angleToHub.minus(robotPoseSupplier.get().getRotation()).unaryMinus();

        // turret.pointAtAngle(Degrees.of(angleToHub.getDegrees()));

        pid.setSetpoint(MathUtils.clamp(-TurretConstants.turretRangeOneWay.in(Degrees),
                TurretConstants.turretRangeOneWay.in(Degrees), angleToHub.getDegrees()));
        double speed = pid.calculate(turret.getAngle().in(Degrees));
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        turret.setSpeed(speed);
        Logger.recordOutput("Turret/PID_Error", pid.getError());
        Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());
        if (Math.abs(turret.getAngle().in(Degrees)-angleToHub.getDegrees())<5){
            LEDColor = LEDConstants.colorLimeGreen;
        }
        else {
            LEDColor = LEDConstants.colorRed;
        }

        spinUpFlywheel(ShooterConstants.FeedSpeed2.in(RPM));
        m_shooter.setHoodPosition(ShooterConstants.FeedHood2);

        LEDManager.setColor(LEDColor);
    }
    public void spinUpFlywheel(double targetRPM) {

        double targetRPS = targetRPM / 60.0;

        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); // Tuned in V per rot/s
        double controlOutput = flywheelController.calculate(
                m_shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio)
                        .in(RotationsPerSecond),
                targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage
                + controlOutput * ShooterConstants.FlywheelConstants.bangBangCoefficient;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        Logger.recordOutput("Aiming/Flywheel/TargetRPM", targetRPM);
        Logger.recordOutput("Aiming/Flywheel/FeedforwardVoltage", feedforwardVoltage);
        Logger.recordOutput("Aiming/Flywheel/TotalVoltage", totalVoltage);

        m_shooter.targetSpeed = RPM.of(targetRPM);

        m_shooter.setFlywheelVoltage(Volt.of(totalVoltage));
        }

    @Override
    public void end(boolean interrupted) {
        turret.setSpeed(0);
        m_shooter.setHoodPosition(-1d);
        m_shooter.setFlywheelVoltage(Volt.of(0));
        LEDManager.setDefault();
    }
}
