// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;

public class ShootAtHubCommand extends Command {
    @SuppressWarnings("PMD.UnusedPrivateField")
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    private PIDController turretPID = new PIDController(0.008, 0, 0.0001);
    private BangBangController flywheelController = new BangBangController();
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
        ((Measure<PerUnit<VoltageUnit, AngularVelocityUnit>>) ShooterConstants.FlywheelConstants.kS).in(ShooterConstants.FlywheelConstants.VoltsPerRotationsPerSecond),
        ((Measure<PerUnit<VoltageUnit, AngularVelocityUnit>>) ShooterConstants.FlywheelConstants.kV).in(ShooterConstants.FlywheelConstants.VoltsPerRotationsPerSecond)); //Safe
    private final Pose2d targetHubPose;

    public ShootAtHubCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
            Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.targetHubPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? TurretConstants.hubBlue: TurretConstants.hubRed;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        AimingConstants.initialize();
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedSupplier.get();

        Pose2d turretPose = robotPose.transformBy(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        Translation2d relVelocity = new Translation2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond);

        Translation2d relPosition = targetHubPose.getTranslation().minus(turretPose.getTranslation());
        double distanceToTarget = relPosition.getNorm();

        double targetRPM = AimingConstants.flywheelSpeedMap.get(distanceToTarget);
        double TOF = AimingConstants.flywheelTOFMap.get(distanceToTarget);
        double oldRPM = targetRPM;

        for (int i = 0; i < AimingConstants.maxIterations; i++) {
            Translation2d predictedTurretTranslation = turretPose.getTranslation().plus(relVelocity.times(TOF));

            Translation2d predictedRelPosition = targetHubPose.getTranslation().minus(predictedTurretTranslation);
            double predictedDistance = predictedRelPosition.getNorm();

            targetRPM = AimingConstants.flywheelSpeedMap.get(predictedDistance);

            double difference = targetRPM - oldRPM;
            if (Math.abs(difference) > AimingConstants.maxRPMChange * TOF) {
                targetRPM = oldRPM + Math.signum(difference) * AimingConstants.maxRPMChange * TOF;
            }

            double newTOF = AimingConstants.flywheelTOFMap.get(predictedDistance);

            if (Math.abs(newTOF - TOF) < AimingConstants.ToFtolerance) {
                break;
            }

            TOF = newTOF;
            oldRPM = targetRPM;
        }

        Pose2d predictedRobotPose = robotPose.plus(
                new Transform2d(
                        new Translation2d(
                                chassisSpeeds.vxMetersPerSecond * TOF,
                                chassisSpeeds.vyMetersPerSecond * TOF),
                        new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * TOF)));

        aimTurret(predictedRobotPose, targetHubPose);
        spinUpFlywheel(targetRPM);
    }

    public void spinUpFlywheel(double targetRPM) {
        double targetRPS = targetRPM / 60.0;
        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); //Tuned in V per rot/s
        double controlOutput = flywheelController.calculate(
                m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio).in(RotationsPerSecond), targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage + controlOutput;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        m_Shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

    public void aimTurret(Pose2d predictedPose, Pose2d Target) {
        Pose2d predictedTurretPose = predictedPose.transformBy(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));
        Rotation2d angleToTarget = Target.getTranslation().minus(predictedTurretPose.getTranslation()).getAngle();
        //angleToTarget = angleToTarget.minus(robotPoseSupplier.get().getRotation());
        angleToTarget = angleToTarget.minus(predictedPose.getRotation());
        double setpoint = MathUtil.clamp(
            angleToTarget.getDegrees(),
            -TurretConstants.turretRangeOneWay.in(Degrees),
             TurretConstants.turretRangeOneWay.in(Degrees)
        );
        turretPID.setSetpoint(setpoint);
        double speed = turretPID.calculate(m_Turret.inputs.turretAngle.in(Degrees));
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        m_Turret.setSpeed(speed);

        Logger.recordOutput("Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Turret/PID_Setpoint", turretPID.getSetpoint());
    }

    public void setHoodAngle(Angle a) {
        m_Shooter.setHoodAngle(a);
    }

    @Override
    public void end(boolean interrupted) {
        m_Turret.setSpeed(0);
        m_Shooter.setFlywheelVoltage(Volt.of(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private class AimingConstants {
        public static final int maxIterations = 4;
        public static final double ToFtolerance = 0.05; // seconds
        public static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap(); // Meters to  RPM
        public static final InterpolatingDoubleTreeMap flywheelTOFMap = new InterpolatingDoubleTreeMap(); // Meters toseconds in air
        public static final double maxRPMChange = 300; // RPM per second TODO: tune this

        public static void initialize() { //TODO: fill out once bot is done
            // Meters to RPM
            flywheelSpeedMap.put(Meters.of(0).in(Meters), RPM.of(0).in(RPM));

            // Meters to tof
            flywheelTOFMap.put(Meters.of(0).in(Meters), Seconds.of(0).in(Seconds));
        }

        public AimingConstants() throws Exception {
            throw new Exception("dont instantiate this");
        }
    }
}
