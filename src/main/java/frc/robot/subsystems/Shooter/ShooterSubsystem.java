package frc.robot.subsystems.Shooter;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase{
    private final ShooterFlywheelIO flywheelIO;
    private final ShooterHoodIO hoodIO;

    private final ShooterFlywheelIOInputsAutoLogged flywheelInputs = new ShooterFlywheelIOInputsAutoLogged();
    private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

    private final SysIdRoutine flywheelSysIdRoutine;

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
