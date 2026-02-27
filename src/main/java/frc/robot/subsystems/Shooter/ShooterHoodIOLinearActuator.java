package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.util.MathUtils;

public class ShooterHoodIOLinearActuator implements ShooterHoodIO {
    private Servo actuator;
    private Servo actuator2;

    public ShooterHoodIOLinearActuator(int pwmID, int pwmID2) {
        actuator = new Servo(pwmID);
        actuator2 = new Servo(pwmID2);

        actuator.setBoundsMicroseconds((int) (2.0 * 1000), (int) (1.8 * 1000), (int) (1.5 * 1000), (int) (1.2 * 1000),
                (int) (1.0 * 1000));
        actuator2.setBoundsMicroseconds((int) (2.0 * 1000), (int) (1.8 * 1000), (int) (1.5 * 1000), (int) (1.2 * 1000),
                (int) (1.0 * 1000));
    }

    @Override
    public void setPosition(double position) {
        System.out.println("Position: " + position);
        position = MathUtils.clamp(-1d, 1d, position);
        actuator.setSpeed(position);
        actuator2.setSpeed(position);
    }

    public void setLength(Distance length) {
        length = length.minus(ShooterConstants.HoodConstants.actuatorLengthRetracted);
        System.out.println("Length: " + length);
        if (length.gt(ShooterConstants.HoodConstants.actuatorLengthExtended)) {
            length = ShooterConstants.HoodConstants.actuatorLengthExtended;
        } else if (length.lt(Meter.of(0))) {
            length = Meter.of(0);
        }

        double command = (length.in(Meter) / ShooterConstants.HoodConstants.actuatorLengthExtended.in(Meter)) * 2 - 1;
        setPosition(command);
    }

    @Override
    public void updateInputs(ShooterHoodIOInputs inputs) {

        double normalizedPosition = actuator.getSpeed(); // -1 to 1

        double actuatorLength = ShooterConstants.HoodConstants.actuatorLengthRetracted.in(Meter)
                + (normalizedPosition + 1) / 2 * (ShooterConstants.HoodConstants.actuatorLengthExtended.in(Meter) - ShooterConstants.HoodConstants.actuatorLengthRetracted.in(Meter));
        double a = ShooterConstants.HoodConstants.hoodPivotToActuatorMount.in(Meter);
        double b = ShooterConstants.HoodConstants.hoodPivotToHoodEdge.in(Meter);

        double cosTheta = (a * a + b * b - actuatorLength * actuatorLength) / (2 * a * b);

        cosTheta = MathUtils.clamp(-1.0, 1.0, cosTheta); // prevent Nan

        double theta = Math.acos(cosTheta);

        theta += ShooterConstants.HoodConstants.angleOffset.in(Radian);

        inputs.angle = Radian.of(theta); // Ground relative
        inputs.position = normalizedPosition;
    }

    @Override
    public void setAngle(Angle a) { //Absolute from ground

        double theta = a.in(Radian)
                - ShooterConstants.HoodConstants.angleOffset.in(Radian);

        theta = MathUtils.clamp(
                ShooterConstants.HoodConstants.minAngle.in(Radian),
                ShooterConstants.HoodConstants.maxAngle.in(Radian),
                theta);

        double A = ShooterConstants.HoodConstants.hoodPivotToActuatorMount.in(Meter);
        double B = ShooterConstants.HoodConstants.hoodPivotToHoodEdge.in(Meter);

        double lengthSquared = A * A + B * B - 2 * A * B * Math.cos(theta);

        double actuatorLength = Math.sqrt(Math.max(0, lengthSquared));

        setLength(Meter.of(actuatorLength));
    }
}

