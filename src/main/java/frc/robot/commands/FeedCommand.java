package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;

public class FeedCommand extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private final ShooterSubsystem m_shooter; 
    private final Supplier<Pose2d> target;
    private PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);

    public FeedCommand(TurretSubsystem turret, ShooterSubsystem shooter, Pose2d target, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turret = turret;
        this.m_shooter = shooter;
        this.robotPoseSupplier = robotPoseSupplier;
        this.target = () -> target;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        addRequirements(turret);
    }

    public FeedCommand(TurretSubsystem turret, ShooterSubsystem shooter, Supplier<Pose2d> target, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turret = turret;
        this.m_shooter = shooter;
        this.robotPoseSupplier = robotPoseSupplier;
        this.target = target;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        addRequirements(turret);
    }


    @Override
    public void initialize() {
        pid.setTolerance(TurretConstants.tolerance);
    }

    @Override
    public void execute() {
        Pose2d turretPose = robotPoseSupplier.get()
                .transformBy(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        ChassisSpeeds robotVelocity = chassisSpeedSupplier.get();
        Pose2d predictedTurretPose = turretPose.plus(
                new Transform2d(
                        new Translation2d(
                                robotVelocity.vxMetersPerSecond * TurretConstants.lookaheadLatency.in(Seconds),
                                robotVelocity.vyMetersPerSecond * TurretConstants.lookaheadLatency.in(Seconds)),
                        new Rotation2d(robotVelocity.omegaRadiansPerSecond * TurretConstants.lookaheadLatency.in(Seconds))));
        Rotation2d angleToHub = target.get().getTranslation().minus(predictedTurretPose.getTranslation()).getAngle();
        angleToHub = angleToHub.minus(robotPoseSupplier.get().getRotation()).unaryMinus();
        
        //turret.pointAtAngle(Degrees.of(angleToHub.getDegrees()));

        pid.setSetpoint(MathUtils.clamp(-TurretConstants.turretRangeOneWay.in(Degrees), TurretConstants.turretRangeOneWay.in(Degrees), angleToHub.getDegrees()));
        double speed = pid.calculate(turret.getAngle().in(Degrees));
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        turret.setSpeed(speed);
        Logger.recordOutput("Turret/PID_Error", pid.getError());
        Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());

        m_shooter.setFlywheelVelocity(ShooterConstants.FeedSpeed);
        m_shooter.setHoodPosition(ShooterConstants.FeedHood);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setSpeed(0);
    }

}
