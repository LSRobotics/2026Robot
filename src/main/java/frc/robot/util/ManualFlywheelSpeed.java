package frc.robot.util;

import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.measure.AngularVelocity;
import frc.robot.subsystems.Shooter.ShooterConstants.FlywheelConstants;

public class ManualFlywheelSpeed {
    public static AngularVelocity speed = FlywheelConstants.manualSpeed1;

    public static void setSpeed(AngularVelocity value) {
        speed = value;
        Logger.recordOutput("Manual/speed", speed);
    }

    public static void init(){
        Logger.recordOutput("Manual/speed", speed);
    }

    public static AngularVelocity getSpeed() {
        return speed;
    }
}
