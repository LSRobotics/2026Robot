package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;
// Based on https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/#wheel-radius-characterization
//Updat e src/main/deploy/pathplanner/settings.json ("driveWheelRadius") and TunerConstants.kWheelRadius with output
public class WheelRadiusCommand extends Command {

    private final CommandSwerveDrivetrain m_swerve;

    private final Timer timer = new Timer();

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // rad/Sec^2
    public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    private final SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);

    private final SwerveRequest.ApplyRobotSpeeds speedsRequest = new SwerveRequest.ApplyRobotSpeeds();

    private double[] startPositions = new double[4];
    private Rotation2d lastAngle = Rotation2d.kZero;
    private double gyroDelta = 0.0;

    private boolean measuring = false;

    public WheelRadiusCommand(CommandSwerveDrivetrain swerve) {
        this.m_swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        limiter.reset(0.0);

        timer.reset();
        timer.start();
        measuring = false;

        gyroDelta = 0.0;
        lastAngle = m_swerve.getPigeon2().getRotation2d();
        m_swerve.applyRequest(()->new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d()));
    }

    @Override
    public void execute() {
        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
        
        m_swerve.applyRequest(()-> speedsRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, speed)));

        if (!measuring && timer.get() > 1.5) {
            startPositions = getWheelPositions(m_swerve);
            lastAngle = m_swerve.getPigeon2().getRotation2d();
            gyroDelta = 0.0;
            measuring = true;
        }

        if (measuring) {
            Rotation2d rotation = m_swerve.getPigeon2().getRotation2d();
            gyroDelta += Math.abs(rotation.minus(lastAngle).getRadians());
            lastAngle = rotation;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        m_swerve.applyRequest(()-> speedsRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));

        if (!measuring) return;

        double[] positions = getWheelPositions(m_swerve);

        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(positions[i] - startPositions[i]) / 4.0;
        }

        double wheelRadius =(gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;

        NumberFormat formatter = new DecimalFormat("#0.000");

        System.out.println("********** Wheel Radius Characterization Results **********");
        System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
        System.out.println("\tGyro Delta: " + formatter.format(gyroDelta) + " radians");
        System.out.println("\tWheel Radius: "
                + formatter.format(wheelRadius) + " meters, "
                + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(gyroDelta) > 2* (2 * Math.PI); // stop after 2 rot
    }

    public double[] getWheelPositions(CommandSwerveDrivetrain swerve) {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerve.getModule(i).getCurrentState().angle.getRadians();
        }
        return values;
    }
}