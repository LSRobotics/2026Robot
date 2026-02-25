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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;

public class AimAtHubCommand extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);

    public AimAtHubCommand(TurretSubsystem turret, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turret = turret;
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
        Pose2d hubPose = TurretConstants.hubBlue;
        Pose2d turretPose = robotPoseSupplier.get()
                .transformBy(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        ChassisSpeeds robotVelocity = chassisSpeedSupplier.get();
        Pose2d predictedTurretPose = turretPose.plus(
                new Transform2d(
                        new Translation2d(
                                robotVelocity.vxMetersPerSecond * TurretConstants.lookaheadLatency.in(Seconds),
                                robotVelocity.vyMetersPerSecond * TurretConstants.lookaheadLatency.in(Seconds)),
                        new Rotation2d(robotVelocity.omegaRadiansPerSecond * TurretConstants.lookaheadLatency.in(Seconds))));
        Rotation2d angleToHub = hubPose.getTranslation().minus(predictedTurretPose.getTranslation()).getAngle();
        angleToHub = angleToHub.minus(robotPoseSupplier.get().getRotation());
        int trajectoryPoints = 20;
        Pose2d[] trajectory = new Pose2d[trajectoryPoints];
        for (int i = 0; i < trajectoryPoints; i++) {
            double t = (double) i / (trajectoryPoints - 1);
            trajectory[i] = new Pose2d(predictedTurretPose.getTranslation(), new Rotation2d())
                    .interpolate(new Pose2d(hubPose.getTranslation(), new Rotation2d()), t);
        }

        Logger.recordOutput("Turret/Trajectory", trajectory);
        Logger.recordOutput("Turret/AngleToHub", angleToHub.getDegrees());
        Logger.recordOutput("Turret/PredictedTurretPose", predictedTurretPose);

        Rotation2d actualFieldAngle = Rotation2d.fromDegrees(turret.getAngle().in(Degrees))
                .plus(robotPoseSupplier.get().getRotation());
        Logger.recordOutput("Field Angle", actualFieldAngle);

        Logger.recordOutput("Turret/TargetRay", new Pose2d[] {
                turretPose,
                new Pose2d(TurretConstants.hubBlue.getTranslation(), angleToHub)
        });

        Logger.recordOutput("Turret/ActualRay", new Pose2d[] {
                turretPose,
                new Pose2d(turretPose.getTranslation().plus(new Translation2d(5.0, actualFieldAngle)), actualFieldAngle)
        });

        //turret.pointAtAngle(Degrees.of(angleToHub.getDegrees()));
        pid.setSetpoint(angleToHub.getDegrees());
        double speed = pid.calculate(turret.inputs.turretAngle.in(Degrees));
        SmartDashboard.putNumber("Angle2", angleToHub.getDegrees());
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        turret.setSpeed(speed);
        Logger.recordOutput("Turret/PID_Error", pid.getError());
        Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        turret.setSpeed(0);
    }

}
