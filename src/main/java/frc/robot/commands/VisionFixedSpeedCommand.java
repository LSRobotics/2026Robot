package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootAtHubCommand.AimingConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.util.MathUtils;
import frc.robot.util.ShotSolution;

public class VisionFixedSpeedCommand extends Command{
    
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Supplier<ShotSolution> m_Shot;

    //private PIDController turretPID = new PIDController(0.008, 0, 0.0001);
    private PIDController turretPID = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    private BangBangController flywheelController = new BangBangController();
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private Pose2d targetHubPose = TurretConstants.hubBlue;
    //@SuppressWarnings("unchecked")
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
        ShooterConstants.FlywheelConstants.kS.in(Volts),
        ShooterConstants.FlywheelConstants.kV, 
        ShooterConstants.FlywheelConstants.kA);

    public VisionFixedSpeedCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,ShotSolution shot, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.m_Shot = () -> shot;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
    }

        public VisionFixedSpeedCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,Supplier<ShotSolution> shot, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.m_Shot = shot;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
    }

    @Override
    public void execute(){
        setHoodAngle(m_Shot.get().hoodAngle());
        spinUpFlywheel(m_Shot.get().targetRPM());
                Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedSupplier.get();

        Pose2d turretPose = robotPose.transformBy(
                new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        Translation2d robotVelocity = new Translation2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

        Translation2d turretOffset = TurretConstants.turretOffset;
        Translation2d tangentialVelocity = new Translation2d(
                -chassisSpeeds.omegaRadiansPerSecond * turretOffset.getY(),
                chassisSpeeds.omegaRadiansPerSecond * turretOffset.getX());

        Translation2d relVelocity = robotVelocity.plus(tangentialVelocity);

        Translation2d relPosition = targetHubPose.getTranslation()
                .minus(turretPose.getTranslation());
        double distanceToTarget = relPosition.getNorm();

        Logger.recordOutput("Aiming/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("Aiming/TargetHubPose", targetHubPose);
        Logger.recordOutput("Aiming/TurretPose", turretPose);

        double targetRPM = AimingConstants.flywheelSpeedMap.get(distanceToTarget);
        double TOF = AimingConstants.flywheelTOFMap.get(distanceToTarget);
        double oldRPM = targetRPM;
        double predictedDistance = distanceToTarget;

        for (int i = 0; i < AimingConstants.maxIterations; i++) {
            Translation2d predictedTurretTranslation = turretPose.getTranslation().plus(relVelocity.times(TOF));

            Translation2d predictedRelPosition = targetHubPose.getTranslation().minus(predictedTurretTranslation);
            predictedDistance = predictedRelPosition.getNorm();

            targetRPM = AimingConstants.flywheelSpeedMap.get(predictedDistance);
            Logger.recordOutput("Aiming/RawSpeedRPM", targetRPM);

            double difference = targetRPM - oldRPM;
            if (Math.abs(difference) > AimingConstants.maxRPMChange * TOF) {
                targetRPM = oldRPM + Math.signum(difference) * AimingConstants.maxRPMChange * TOF;
            }

            double newTOF = AimingConstants.flywheelTOFMap.get(predictedDistance);

            Logger.recordOutput("Aiming/Iteration", i);
            Logger.recordOutput("Aiming/PredictedDistance", predictedDistance);
            Logger.recordOutput("Aiming/TargetRPM", targetRPM);
            Logger.recordOutput("Aiming/TOF", TOF);

            if (Math.abs(newTOF - TOF) < AimingConstants.ToFtolerance) {
                break;
            }

            TOF = newTOF;
            oldRPM = targetRPM;
        }

        Translation2d predictedTurretTranslation = turretPose.getTranslation().plus(relVelocity.times(TOF));

        aimTurret(predictedTurretTranslation, robotPoseSupplier.get().getRotation(), this.targetHubPose);
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

 public void aimTurret(Translation2d predictedTurretPosition, Rotation2d robotRotation, Pose2d target) {
        Translation2d toTarget = target.getTranslation().minus(predictedTurretPosition);

        Rotation2d fieldAngle = toTarget.getAngle();

        Rotation2d turretAngle = fieldAngle.minus(robotRotation).unaryMinus();

        double setpoint = MathUtil.clamp(
                turretAngle.getDegrees(),
                -TurretConstants.turretRangeOneWay.in(Degrees),
                TurretConstants.turretRangeOneWay.in(Degrees));

        turretPID.setSetpoint(setpoint);

        double speed = turretPID.calculate(m_Turret.getAngle().in(Degrees));
        Logger.recordOutput("Aiming/Turret/Unclamped Speed", speed);

        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);

        m_Turret.setSpeed(speed);

        Logger.recordOutput("Aiming/Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Aiming/Turret/Speed", speed);
        Logger.recordOutput("Aiming/Turret/PID_Setpoint", turretPID.getSetpoint());

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
    public void initialize(){
                this.targetHubPose = DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Blue
        ? TurretConstants.hubBlue
        : TurretConstants.hubRed;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
