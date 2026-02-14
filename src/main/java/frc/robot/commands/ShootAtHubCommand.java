// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volt;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.util.MathUtils;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShootAtHubCommand extends Command {
    @SuppressWarnings("PMD.UnusedPrivateField")
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    private  PIDController turretPID = new PIDController(0.008, 0, 0.0001);
    private BangBangController flywheelController = new BangBangController();
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.FlywheelConstants.kS, ShooterConstants.FlywheelConstants.kV);
    private final Pose2d targetHubPose;

  
    public ShootAtHubCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.targetHubPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? TurretConstants.hubBlue : TurretConstants.hubRed;
        addRequirements(m_Turret, m_Shooter);
    }

    @Override
    public void initialize() {
        ShootingConstants.initialize();
        turretPID.setTolerance(0.2);
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedSupplier.get();

        Translation2d relPosition = targetHubPose.getTranslation().minus(robotPose.getTranslation());
        Translation2d relVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        double distanceToTarget = relPosition.getNorm();
        double targetRPM = ShootingConstants.flywheelSpeedMap.get(distanceToTarget);
        double timeOfFlight = ShootingConstants.flywheelTOFMap.get(targetRPM);
        double oldRPM = targetRPM;

        for (int i = 0; i < ShootingConstants.maxIterations; i++) {
            Translation2d predictedRelPosition = relPosition.plus(relVelocity.times(timeOfFlight));
            targetRPM = ShootingConstants.flywheelSpeedMap.get(predictedRelPosition.getNorm());
            double difference = targetRPM - oldRPM;
            if (Math.abs(difference) > ShootingConstants.maxRPMChange * timeOfFlight) {
                targetRPM = oldRPM + Math.signum(difference) * ShootingConstants.maxRPMChange * timeOfFlight;
            }

            double newTimeOfFlight = ShootingConstants.flywheelTOFMap.get(targetRPM);
            if (Math.abs(newTimeOfFlight - timeOfFlight) < ShootingConstants.ToFtolerance) {
                break;
            }
            timeOfFlight = newTimeOfFlight;
            oldRPM = targetRPM;
        }

        aimTurret(robotPose, targetHubPose);
        spinUpFlywheel(targetRPM);
    }

    public void spinUpFlywheel(double targetRPM) {
        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPM);
        double controlOutput = flywheelController.calculate(m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio).in(RPM), targetRPM)*ShooterConstants.FlywheelConstants.maxVoltage.in(Volt);
        double totalVoltage = feedforwardVoltage + controlOutput;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt), ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));
        m_Shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

    public void aimTurret(Pose2d predictedPose, Pose2d Target) {
        Pose2d predictedTurretPose = predictedPose.transformBy(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));
        Rotation2d angleToTarget = predictedPose.getTranslation().minus(predictedTurretPose.getTranslation()).getAngle();
        angleToTarget = angleToTarget.minus(robotPoseSupplier.get().getRotation());

        turretPID.setSetpoint(angleToTarget.getDegrees());
        double speed = turretPID.calculate(m_Turret.inputs.turretAngle.in(Degrees));
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        m_Turret.setSpeed(speed);

        Logger.recordOutput("Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Turret/PID_Setpoint", turretPID.getSetpoint());
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


    private class ShootingConstants {
        public static final int maxIterations = 5;
        public static final double ToFtolerance = 0.5; //seconds 
        public static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap(); //Flywheel speed in rpm to distance in meters
        public static final InterpolatingDoubleTreeMap flywheelTOFMap = new InterpolatingDoubleTreeMap(); //Flywheel speed in rpm to time of flight in seconds
        public static final double maxRPMChange = 300; //RPM per second TODO: tune this
        public static void initialize() {
            //RPM to Distance(M)
            flywheelSpeedMap.put(0.0, 0.0);

            //RPM to ToF
            flywheelTOFMap.put(0.0, 0.0);
        }
        public ShootingConstants() throws Exception {
             throw new Exception("dont instantiate this class");
        }
    }
}
