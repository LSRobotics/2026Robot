package frc.robot.commands;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RunFlywheelCommandSpeed extends Command {
    private ShooterSubsystem m_Shooter;
    private DoubleSupplier m_Speed;


    public RunFlywheelCommandSpeed(ShooterSubsystem shooter, DoubleSupplier speed) {
        m_Shooter = shooter;
        m_Speed = speed;
        addRequirements(m_Shooter);

    }

    @Override
    public void initialize() {
    }

    public void execute(){
        m_Shooter.setFlywheelSpeed(m_Speed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
