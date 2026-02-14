package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;

public class ShooterHoodIOLinearActuator implements ShooterHoodIO{
    private Servo actuator;

    public ShooterHoodIOLinearActuator(int pwmID) {
        actuator = new Servo(pwmID);

        actuator.setBoundsMicroseconds((int)(2.0*1000), (int)(1.8*1000), (int)(1.5*1000), (int)(1.2*1000), (int)(1.0*1000));
    }

    @Override
    public void setPosition(double position) {
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

}
