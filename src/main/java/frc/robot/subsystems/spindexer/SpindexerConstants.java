package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 60; // TODO: Set the actual motor ID
    public static final double SPINDEXER_SPEED = 0.5;
    public static final double SPINDEXER_MIN_SPEED = 0.45;

    public static final Current spindexerJamThreshold = Amps.of(10); // TODO: Tune
    public static final Time jamRecoveryTime = Seconds.of(0.5);
    public static final double jamRecoverySpeed = -0.3; // TODO
    
}
