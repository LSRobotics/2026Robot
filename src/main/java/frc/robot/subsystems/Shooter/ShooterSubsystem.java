package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final ShooterFlywheelIO flywheelIO;
    private final ShooterHoodIO hoodIO;

    private final ShooterFlywheelIO.ShooterFlywheelIOInputs flywheelInputs = new ShooterFlywheelIO.ShooterFlywheelIOInputs();
    //private final ShooterHoodIO.ShooterHoodIOInputs hoodInputs = new ShooterH

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

    public AngularVelocity getFlywheelVelocity() {
        return RPM.of(0);
    }

}
