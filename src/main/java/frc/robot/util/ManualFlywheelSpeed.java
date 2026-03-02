package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;

public class ManualFlywheelSpeed {
    public static double speed = FlywheelConstants.manualSpeed1;

    public static void setSpeed(double value) {
        speed = value;
        Logger.recordOutput("Manual/speed", speed);
    }

    public static double getSpeed() {
        return speed;
    }
}
