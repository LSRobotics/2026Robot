package frc.robot.commands;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volt;

import java.util.function.DoubleSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.Command;
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
