package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 60; // TODO: Set the actual motor ID
    public static final double SPINDEXER_SPEED = 0.7; //0.575, 0.65 (Best)
    public static final double SPINDEXER_MIN_SPEED = 0.35;

    public static final Current spindexerJamThreshold = Amps.of(40); // TODO: Tune
    public static final Time jamRecoveryTime = Seconds.of(0.00);

    public static final double jamRecoverySpeed = 0; 
    public static final double spindexerRampRatePositive = 0.1; // per second
    public static final double spindexerRampRateNegative = -spindexerRampRatePositive; // per second
    
}
