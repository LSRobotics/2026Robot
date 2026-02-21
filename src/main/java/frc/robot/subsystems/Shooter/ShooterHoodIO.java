package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface ShooterHoodIO {
    @AutoLog
    public class ShooterHoodIOInputs {
        public double position = 0;
    }

    public default void setPosition(double position) {}

    public default void setLength(Distance length) {}

    public default void setAngle(Angle angle) {}

    public void updateInputs(ShooterHoodIOInputs inputs);

}
