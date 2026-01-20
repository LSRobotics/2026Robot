package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = -1; // TODO: Set the actual motor ID
    public static final double INTAKE_SPEED = 0.7;
    public static final double INTAKE_ARM_SPEED = 0.5;
    public static final double OUTTAKE_SPEED = -INTAKE_SPEED;
    public static final Angle INTAKE_ARM_REST_ANGLE = Angle.ofBaseUnits(0, Degree);
    public static final Angle INTAKE_ARM_DEPLOY_ANGLE = Angle.ofBaseUnits(90, Degree);
}
