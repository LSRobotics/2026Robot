package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterSubsystem {
    private final ShooterFlywheelIO flywheelIO;
    private final ShooterHoodIO hoodIO;

    public ShooterSubsystem(ShooterFlywheelIO flywheelIO, ShooterHoodIO hoodIO) {
        this.flywheelIO = flywheelIO;
        this.hoodIO = hoodIO;
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelIO.setVelocity(velocity);
    }

    public void setFlywheelVoltage(Voltage voltage) {
        flywheelIO.setVoltage(voltage);
    }

    public void setFlywheelSpeed(double speed) {
        flywheelIO.setSpeed(speed);
    }

    public void setHoodPosition(double position) {
        hoodIO.setPosition(position);
    }

}
