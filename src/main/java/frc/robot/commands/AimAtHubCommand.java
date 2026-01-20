package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class AimAtHubCommand extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    public AimAtHubCommand(TurretSubsystem turret, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
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
                                robotVelocity.vxMetersPerSecond * 0.2,
                                robotVelocity.vyMetersPerSecond * 0.2),
                        new Rotation2d(robotVelocity.omegaRadiansPerSecond * 0.2)));
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

        Logger.recordOutput("Turret/TargetRay", new Pose2d[] {
                turretPose,
                new Pose2d(TurretConstants.hubBlue.getTranslation(), angleToHub)
        });

        Logger.recordOutput("Turret/ActualRay", new Pose2d[] {
                turretPose,
                new Pose2d(turretPose.getTranslation().plus(new Translation2d(2.0, actualFieldAngle)), actualFieldAngle)
        });

        turret.pointAtAngle(Degrees.of(angleToHub.getDegrees()));
    }

    @Override
    public void end(boolean interrupted) {
    }

}
