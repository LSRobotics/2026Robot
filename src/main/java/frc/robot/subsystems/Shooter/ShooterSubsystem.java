package frc.robot.subsystems.Shooter;


import static org.wpilib.units.Units.DegreesPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase{
    private final ShooterFlywheelIO flywheelIO;
    private final ShooterHoodIO hoodIO;

    private final ShooterFlywheelIOInputsAutoLogged flywheelInputs = new ShooterFlywheelIOInputsAutoLogged();
    private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

    private final SysIdRoutine flywheelSysIdRoutine;

    public AngularVelocity targetSpeed = DegreesPerSecond.zero();

    public ShooterSubsystem(ShooterFlywheelIO flywheelIO, ShooterHoodIO hoodIO) {
        this.flywheelIO = flywheelIO;
        this.hoodIO = hoodIO;

    flywheelSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            state -> Logger.recordOutput("Shooter/FlywheelSysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            this::setFlywheelVoltage,
            null,
            this));
    }

    @Override
    public void periodic() {
        flywheelIO.updateInputs(flywheelInputs);
        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);
        Logger.processInputs("Shooter/Hood", hoodInputs);
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

    public void setHoodPosition(DoubleSupplier position) {
        hoodIO.setPosition(position.getAsDouble());
    }

    public void setHoodLength(Distance length) {
        hoodIO.setLength(length);
    }

    public void setHoodAngle(Angle angle) {
        hoodIO.setAngle(angle);
    }

    public AngularVelocity getFlywheelVelocity() {
        return flywheelInputs.velocity;
    }

    public Command sysIdFlywheelQuasistatic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdFlywheelDynamic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.dynamic(direction);
    }

}
