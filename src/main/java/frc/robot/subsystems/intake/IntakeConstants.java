package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degree;
import org.wpilib.units.measure.Current;

import org.wpilib.units.measure.Angle;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 52; 
    public static final double INTAKE_IN_SPEED = -1d;
    public static final double OUTTAKE_SPEED = -INTAKE_IN_SPEED;
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
}
