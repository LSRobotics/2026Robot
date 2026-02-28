package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class ArmConstants {
    public static class ArmMotorConstants {
        public static final int ARM_MOTOR_ID = 50;
        public static final int ARM_MOTOR_FOLLOWER_ID = 51;
        public static final double ARM_SPEED = 0.2;
        public static final Angle ARM_REST_ANGLE = Degrees.of(0);
        public static final Angle ARM_DEPLOY_ANGLE = Degrees.of(-25.143);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(10);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(15);
        public static final double ARM_TOLERANCE = 1;
        public static final double gearRatio = 86.4;
    }
    public static class ArmLimitSwitchConstants {
        public static final int LIMIT_SWITCH_ID = 0;
    }
}
