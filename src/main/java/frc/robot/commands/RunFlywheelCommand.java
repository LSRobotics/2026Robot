package frc.robot.commands;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

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

public class RunFlywheelCommand extends Command {
    private ShooterSubsystem m_Shooter;
    private AngularVelocity m_Speed;

    private BangBangController flywheelController = new BangBangController();
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
        ((Measure<PerUnit<VoltageUnit, AngularVelocityUnit>>) ShooterConstants.FlywheelConstants.kS).in(ShooterConstants.FlywheelConstants.VoltsPerRotationsPerSecond),
        ((Measure<PerUnit<VoltageUnit, AngularVelocityUnit>>) ShooterConstants.FlywheelConstants.kV).in(ShooterConstants.FlywheelConstants.VoltsPerRotationsPerSecond)); //Safe

    public RunFlywheelCommand(ShooterSubsystem shooter, AngularVelocity speed) {
        m_Shooter = shooter;
        m_Speed = speed;

        addRequirements(m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
    }

    @Override
    public void initialize() {
    }

    public void execute(){
        double targetRPS = m_Speed.in(RotationsPerSecond) / ShooterConstants.FlywheelConstants.gearRatio; 
        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); //Tuned in V per rot/s
        double controlOutput = flywheelController.calculate(
                m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio).in(RotationsPerSecond), targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage + controlOutput;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        m_Shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
