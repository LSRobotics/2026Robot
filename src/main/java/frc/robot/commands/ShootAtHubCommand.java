
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    //private PIDController turretPID = new PIDController(0.008, 0, 0.0001);
    private PIDController turretPID = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    private BangBangController flywheelController = new BangBangController();
    //@SuppressWarnings("unchecked")
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
        ShooterConstants.FlywheelConstants.kS.in(Volts),
        ShooterConstants.FlywheelConstants.kV, 
        ShooterConstants.FlywheelConstants.kA);
    private final Pose2d targetHubPose;

    public ShootAtHubCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
            Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.targetHubPose = DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Blue ? TurretConstants.hubBlue: TurretConstants.hubRed;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        AimingConstants.initialize();
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
        Logger.recordOutput("Aiming/TargetHubPose", targetHubPose);
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

        Pose2d predictedRobotPose = robotPose.plus(
                new Transform2d(
                        new Translation2d(
                                chassisSpeeds.vxMetersPerSecond * TOF,
                                chassisSpeeds.vyMetersPerSecond * TOF),
                        new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * TOF)));

                        double hoodAngleDeg = AimingConstants.hoodAngleMap.get(predictedDistance);

        setHoodAngle(hoodAngleDeg);
        aimTurret(predictedRobotPose, targetHubPose);
        spinUpFlywheel(targetRPM);
    }

    public void spinUpFlywheel(double targetRPM) {
        double targetRPS = targetRPM / 60.0;


        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); //Tuned in V per rot/s
        double controlOutput = flywheelController.calculate( 
                m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio).in(RotationsPerSecond), targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage + controlOutput*0.25;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        Logger.recordOutput("Aiming/Flywheel/TargetRPM", targetRPM);
        Logger.recordOutput("Aiming/Flywheel/FeedforwardVoltage", feedforwardVoltage);
        Logger.recordOutput("Aiming/Flywheel/TotalVoltage", totalVoltage);

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

        Logger.recordOutput("Aiming/Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Aiming/Turret/PID_Setpoint", turretPID.getSetpoint());
    }

    public void setHoodAngle(double a) {
        m_Shooter.setHoodPosition(a);
        Logger.recordOutput("Aiming/Hood/Angle", a);
    }

    @Override
    public void end(boolean interrupted) {
        m_Turret.setSpeed(0);
        m_Shooter.setFlywheelVoltage(Volt.of(0));
        m_Shooter.setHoodPosition(-1d);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

        /*TODO: FInalize
    1.Start with mid hood angle

    2.Adjust hood until trajectory is good (Prefer higher arcs and lower rpm ) 

    3.Adjust RPM until consistent

    4. test 5 balls

    5. Measure average TOF (From leaving flywheel to passing top plane of hub funnel)

    6.Populate all 3 tables together
    */
   //First: 2m, 3m, 4m, 5m
   //Second: 0.5m, 1m, 1.5m, 2.5m, 3.5m, 4.5m, 5.5m, 6m
   //Third: 2.25m, 2.75m, 3.25m, 3.75m, 4.25m

    private class AimingConstants {
        public static final int maxIterations = 4;
        public static final double ToFtolerance = 0.05; // seconds
        public static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap(); // Meters to  RPM at best hood angle
        public static final InterpolatingDoubleTreeMap flywheelTOFMap = new InterpolatingDoubleTreeMap(); // Meters toseconds in air at best hood angle
        public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap(); // Meters to hood position as percent from center
        public static final double maxRPMChange = 1000; // RPM per second TODO: tune this

       // public static final double hoodTestRPM = 3000; //TODO: RPM for hood table

        public static void initialize() { //TODO: fill out once bot is done
            // Meters to RPM
            flywheelSpeedMap.put(Meters.convertFrom(44, Inches), RPM.convertFrom(40, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(48, Inches), RPM.convertFrom(35, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(78, Inches), RPM.convertFrom(45, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(105, Inches), RPM.convertFrom(44, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(132, Inches), RPM.convertFrom(50, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(139, Inches), RPM.convertFrom(52, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(154, Inches), RPM.convertFrom(52, RotationsPerSecond)); //TODO: Deal with points of inflection

            flywheelSpeedMap.put(Meters.convertFrom(168, Inches), RPM.convertFrom(52, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(208, Inches), RPM.convertFrom(52, RotationsPerSecond));

            flywheelSpeedMap.put(Meters.convertFrom(236, Inches), RPM.convertFrom(54, RotationsPerSecond));





            // Meters to tof
            flywheelTOFMap.put(Meters.convertFrom(44, Inches), Seconds.of(1.14).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(48, Inches), Seconds.of(0.89).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(78, Inches), Seconds.of(1.34).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(105, Inches), Seconds.of(1.28).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(132, Inches), Seconds.of(1.52).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(139, Inches), Seconds.of(1.56).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(154, Inches), Seconds.of(1.52).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(168, Inches), Seconds.of(1.37).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(208, Inches), Seconds.of(1.38).in(Seconds));

            flywheelTOFMap.put(Meters.convertFrom(236, Inches), Seconds.of(1.15).in(Seconds));




            // Meters to hood angle at hoodTestRPM
            hoodAngleMap.put(Meters.convertFrom(44, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(48, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(78, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(105, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(139, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(139, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(154, Inches), -1d);

            hoodAngleMap.put(Meters.convertFrom(208, Inches), -0.45090180360721444);

            hoodAngleMap.put(Meters.convertFrom(208, Inches), -0.2);

            hoodAngleMap.put(Meters.convertFrom(236, Inches), 0.20040080160320642);



        }

        public AimingConstants() throws Exception {
            throw new Exception("dont instantiate this");

        }
    }

}

 