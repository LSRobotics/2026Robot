package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class AimAtHubCommand extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> robotPoseSupplier;

    private final PIDController pid = new PIDController(0.01, 0, 0);

    public AimAtHubCommand(TurretSubsystem turret, Supplier<Pose2d> robotPoseSupplier) {
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d hubPose = TurretConstants.hubBlue;
        Pose2d turretPose = robotPoseSupplier.get().plus(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));
        Rotation2d angleToHub = hubPose.getTranslation().minus(turretPose.getTranslation()).getAngle();
        System.out.println("Angle to Hub: " + angleToHub.getDegrees());

        turret.pointAtAngle(Degrees.of(angleToHub.getDegrees()));
    }
}
