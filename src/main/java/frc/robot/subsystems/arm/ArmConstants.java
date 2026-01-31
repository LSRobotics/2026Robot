package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
    public static final int ARM_MOTOR_ID = -1; // TODO: Set the actual motor ID
    public static final double ARM_SPEED = 0.5;
    public static final Angle ARM_REST_ANGLE = Angle.ofBaseUnits(0, Degree);
    public static final Angle ARM_DEPLOY_ANGLE = Angle.ofBaseUnits(90, Degree);
}
