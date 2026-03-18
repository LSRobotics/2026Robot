package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;
import frc.robot.util.ShotSolution;

public class TakeShotCommand extends Command {

    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final ShotSolution m_Shot;

    //private PIDController turretPID = new PIDController(0.008, 0, 0.0001);
    private PIDController turretPID = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    private BangBangController flywheelController = new BangBangController();
    //@SuppressWarnings("unchecked")
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
        ShooterConstants.FlywheelConstants.kS.in(Volts),
        ShooterConstants.FlywheelConstants.kV, 
        ShooterConstants.FlywheelConstants.kA);

        public TakeShotCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
            ShotSolution shot) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.m_Shot = shot;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
    }

    @Override
    public void execute(){
        setHoodAngle(m_Shot.hoodAngle());
        aimTurret(m_Shot.turretAngle());
        spinUpFlywheel(m_Shot.targetRPM());
    }

      public void spinUpFlywheel(double targetRPM) {
        double targetRPS = targetRPM / 60.0;


        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); //Tuned in V per rot/s
        double controlOutput = flywheelController.calculate( 
                m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio).in(RotationsPerSecond), targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage + controlOutput*FlywheelConstants.bangBangCoefficient;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        Logger.recordOutput("Fixed Shot/Flywheel/TargetRPM", targetRPM);
        Logger.recordOutput("Fixed Shot/Flywheel/FeedforwardVoltage", feedforwardVoltage);
        Logger.recordOutput("Fixed Shot/Flywheel/TotalVoltage", totalVoltage);

        m_Shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

      public void aimTurret(double targetAngleDeg) {
        double setpoint = MathUtil.clamp(
            targetAngleDeg,
            -TurretConstants.turretRangeOneWay.in(Degrees),
             TurretConstants.turretRangeOneWay.in(Degrees)
        );
        turretPID.setSetpoint(setpoint);
        double speed = turretPID.calculate(m_Turret.getAngle().in(Degrees));
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        m_Turret.setSpeed(speed);

        Logger.recordOutput("Fixed Shot/Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Fixed Shot/Turret/PID_Setpoint", turretPID.getSetpoint());
    }

    public void setHoodAngle(double a) {
        m_Shooter.setHoodPosition(a);
        Logger.recordOutput("Fixed Shot/Hood/Angle", a);
    }

    @Override
    public void end(boolean interrupted) {
        m_Turret.setSpeed(0);
        m_Shooter.setFlywheelVoltage(Volts.of(0));
        m_Shooter.setHoodPosition(-1d);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static final class ShotData{
        public static final ShotSolution leftBump = new ShotSolution(-1,RPM.convertFrom(37, RotationsPerSecond),39);
        //public static final ShotSolution feed = new ShotSolution(0.3,RPM.convertFrom(55, RotationsPerSecond),0);
        public static final ShotSolution leftTrench = new ShotSolution(-1, RPM.convertFrom(51, RotationsPerSecond), 72);
        public static final ShotSolution rightTrench = new ShotSolution(-1, RPM.convertFrom(51, RotationsPerSecond), -72);
    }
}
