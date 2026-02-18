package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.MathUtils;

public class ShooterHoodIOLinearActuator implements ShooterHoodIO{
    private Servo actuator;

    public ShooterHoodIOLinearActuator(int pwmID) {
        actuator = new Servo(pwmID);

        actuator.setBoundsMicroseconds((int)(2.0*1000), (int)(1.8*1000), (int)(1.5*1000), (int)(1.2*1000), (int)(1.0*1000));
    }

    @Override
    public void setPosition(double position) {
        position = MathUtils.clamp(-1d, 1d, position);
        actuator.setSpeed(position);
    }

    public void setLength(Distance length) {
        if (length.gt(ShooterConstants.HoodConstants.actuatorLength)) {
            length = ShooterConstants.HoodConstants.actuatorLength;
        } else if (length.lt(Meter.of(0))) {
            length = Meter.of(0);
        }
        double command = (length.in(Meter) / ShooterConstants.HoodConstants.actuatorLength.in(Meter)) * 2 - 1;
        setPosition(command);
    }

    @Override
    public void updateInputs(ShooterHoodIOInputs inputs) {
        inputs.position = actuator.getPosition();
    }

    @Override
    public void setAngle(Angle a) { //law of cosines
        double theta = a.in(Radian) + ShooterConstants.HoodConstants.angleOffset.in(Radian);
        double value = Math.pow(ShooterConstants.HoodConstants.hoodPivotToActuatorMount.in(Meter), 2) + 
            Math.pow(ShooterConstants.HoodConstants.actuatorMountToHoodEdge.in(Meter), 2) - 
            2 * ShooterConstants.HoodConstants.hoodPivotToActuatorMount.in(Meter) * ShooterConstants.HoodConstants.actuatorMountToHoodEdge.in(Meter) * Math.cos(theta);
        setLength(Meter.of(Math.sqrt(Math.max(0, value))));
}
}
