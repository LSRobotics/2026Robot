package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;

public class TurnTurretCommand extends Command {
    private final TurretSubsystem turret;
    private final DoubleSupplier speed;
    public TurnTurretCommand(TurretSubsystem turret, double speed) {
        this.turret = turret;
        this.speed = ()-> speed;
        addRequirements(turret);
    }

    public TurnTurretCommand(TurretSubsystem turret, DoubleSupplier speed){
        this.turret = turret;
        this.speed = speed;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double clampedSpeed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed.getAsDouble());

        if (turret.getAngle().lte(TurretConstants.turretRangeOneWay.unaryMinus()) && clampedSpeed < 0) {
            clampedSpeed = 0;
        } else if (turret.getAngle().gte(TurretConstants.turretRangeOneWay) && clampedSpeed > 0) {
            clampedSpeed = 0;
        }

        turret.setSpeed(clampedSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        turret.setSpeed(0);
    }
    
}
