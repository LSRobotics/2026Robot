package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class ArmConstants {
    public static class ArmMotorConstants {
        public static final int ARM_MOTOR_ID = -1; // TODO: Set the actual motor ID
        public static final double ARM_SPEED = 0.5;
        public static final Angle ARM_REST_ANGLE = Angle.ofBaseUnits(0, Degree);
        public static final Angle ARM_DEPLOY_ANGLE = Angle.ofBaseUnits(90, Degree);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(0);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(0);
        public static final double ARM_TOLERANCE = 2;
    }
    public static class ArmLimitSwitchConstants {
        public static final int LIMIT_SWITCH_ID = -1;
    }
}
