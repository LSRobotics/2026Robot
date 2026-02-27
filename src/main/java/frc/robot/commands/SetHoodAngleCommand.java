package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SetHoodAngleCommand extends Command {
    private ShooterSubsystem m_Shooter;
    private Supplier<Angle> m_Angle;

    public SetHoodAngleCommand(ShooterSubsystem shooter, Angle angle) {
        m_Shooter = shooter;
        m_Angle = ()->angle;

    }

    public SetHoodAngleCommand(ShooterSubsystem shooter, Supplier<Angle> angle) {
        m_Shooter = shooter;
        m_Angle = angle;

    }

    @Override
    public void initialize() {
        m_Shooter.setHoodAngle(m_Angle.get());
        System.out.println(m_Angle.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
