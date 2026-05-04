package frc.robot.subsystems.Shooter;

import static org.wpilib.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public interface ShooterHoodIO {
    @AutoLog
    public class ShooterHoodIOInputs {
        public double position = 0;
        public Angle angle = ShooterConstants.HoodConstants.angleOffset;
    }

    public default void setPosition(double position) {}

    public default void setLength(Distance length) {}

    public default void setAngle(Angle angle) {}

    public void updateInputs(ShooterHoodIOInputs inputs);

}
