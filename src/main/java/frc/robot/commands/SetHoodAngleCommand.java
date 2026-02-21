package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SetHoodAngleCommand extends Command {
    private ShooterSubsystem m_Shooter;
    private Angle m_Angle;

    public SetHoodAngleCommand(ShooterSubsystem shooter, Angle angle) {
        m_Shooter = shooter;
        m_Angle = angle;

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_Shooter.setHoodAngle(m_Angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
