package frc.robot.commands;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volt;
import static org.wpilib.units.Units.Volts;

import java.util.function.Supplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.shuffleboard.SuppliedValueWidget;
import org.wpilib.command2.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.ManualFlywheelSpeed;

public class RunFlywheelCommand extends Command {
    private ShooterSubsystem m_Shooter;
    private Supplier<AngularVelocity> m_Speed;

    private BangBangController flywheelController = new BangBangController();
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
        ShooterConstants.FlywheelConstants.kS.in(Volts),
        ShooterConstants.FlywheelConstants.kV, 
        ShooterConstants.FlywheelConstants.kA); //Safe

    public RunFlywheelCommand(ShooterSubsystem shooter, AngularVelocity speed) {
        m_Shooter = shooter;
        m_Speed = ()->speed;

        addRequirements(m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
    }

      public RunFlywheelCommand(ShooterSubsystem shooter, Supplier<AngularVelocity> speed) {
        m_Shooter = shooter;
        m_Speed = speed;

        addRequirements(m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
    }

    @Override
    public void initialize() {
        
    }

    public void execute(){
        double targetRPS = m_Speed.get().in(RotationsPerSecond) / ShooterConstants.FlywheelConstants.gearRatio; 
        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); //Tuned in V per rot/s
        double controlOutput = flywheelController.calculate( 
                m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio).in(RotationsPerSecond), targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage + controlOutput*0.25;//TODO; DIsabel temp
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        m_Shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_Shooter.setFlywheelSpeed(0);
    }
    
}
