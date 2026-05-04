package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degree;
import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;

public class ArmConstants {
    public static class ArmMotorConstants {
        public static final int ARM_MOTOR_ID = 50;
        public static final int ARM_MOTOR_FOLLOWER_ID = 51;
        public static final double ARM_SPEED = 0.3;
        public static final Angle ARM_REST_ANGLE = Degrees.of(0);
        public static final Angle ARM_DEPLOY_ANGLE = Degrees.of(-17.2);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(35);
        public static final double ARM_TOLERANCE = 0.5;
        public static final double gearRatio = 86.4;
    }

    public static class ArmLimitSwitchConstants {
        public static final int LIMIT_SWITCH_ID = 0;
    }

}
