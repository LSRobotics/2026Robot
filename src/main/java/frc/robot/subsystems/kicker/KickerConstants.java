package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class KickerConstants {
    public static final int KICKER_MOTOR_ID = 61; 
    public static final double KICKER_SPEED = -0.8; 
    public static final double OUTTAKE_SPEED = -KICKER_SPEED;
    public static final Current KickerCurrentLimit = Amps.of(40);
}