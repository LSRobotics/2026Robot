package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {
    public class FlywheelConstants {
        public static final AngularVelocity maxSpeed = RPM.of(-1);// TODO
        public static final AngularVelocity holdSpeed = RPM.of(10); // TODO
        public static final int flywheelMotor1ID = 63;
        public static final Current statorCurrentLimit = Amps.of(40);
        public static final int flywheelMotor2ID = 64;
    }

    public class HoodConstants {
        public static final int hoodLinearActuatorPWMID = 0;
    }
}
