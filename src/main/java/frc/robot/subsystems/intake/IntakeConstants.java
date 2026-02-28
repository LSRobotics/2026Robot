package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import edu.wpi.first.units.measure.Current;

import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 52; 
    public static final double INTAKE_IN_SPEED = -0.8;
    public static final double OUTTAKE_SPEED = -INTAKE_IN_SPEED;
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
}
