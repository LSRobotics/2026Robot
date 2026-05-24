package frc.robot.subsystems.Shooter;


import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volt;
import static org.wpilib.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

import frc.robot.util.MathUtils;

import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.sysid.SysIdRoutine;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.util.MathUtil;

public class ShooterMechanism extends Mechanism{
    private final ShooterFlywheelIO flywheelIO;
    private final ShooterHoodIO hoodIO;

    private final ShooterFlywheelIOInputsAutoLogged flywheelInputs = new ShooterFlywheelIOInputsAutoLogged();
    private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

    private final SysIdRoutine flywheelSysIdRoutine;

    public AngularVelocity targetSpeed = DegreesPerSecond.zero();

    public ShooterMechanism(ShooterFlywheelIO flywheelIO, ShooterHoodIO hoodIO) {
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

    public Command runFlywheel(Supplier<AngularVelocity> speed) {
        return Command.requiring(this)
            .executing(co -> {
                BangBangController flywheelController = new BangBangController();
                SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
                        ShooterConstants.FlywheelConstants.kS.in(Volts),
                        ShooterConstants.FlywheelConstants.kV,
                        ShooterConstants.FlywheelConstants.kA);

                flywheelController.setTolerance(
                        ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));

                while (true) {
                    double targetRPS = speed.get().in(RotationsPerSecond)
                            / ShooterConstants.FlywheelConstants.gearRatio;

                    double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS);
                    double controlOutput = flywheelController.calculate(
                            getFlywheelVelocity()
                                    .times(ShooterConstants.FlywheelConstants.gearRatio)
                                    .in(RotationsPerSecond),
                            targetRPS)
                            * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);

                    double totalVoltage = MathUtils.clamp(
                            -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                            ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                            feedforwardVoltage + controlOutput * ShooterConstants.FlywheelConstants.bangBangCoefficient);

                    setFlywheelVoltage(Volt.of(totalVoltage));
                    co.yield();
                }
            })
            .whenCanceled(() -> setFlywheelSpeed(0))
            .named("Run Flywheel");
    }

    public Command runFlywheel(AngularVelocity speed) {
        return runFlywheel(() -> speed);
    }

    public Command runFlywheel(DoubleSupplier speed) {
        return run(co->{
            while (true){
                 setFlywheelSpeed(speed.getAsDouble());
                 co.yield();
            }
        }).whenCanceled(() -> setFlywheelSpeed(0)).named("Run flywheel (Duty Cycle)");
    }

  public Command stop(){
        return run(co -> {setFlywheelSpeed(0);}).named("Stop Shooter");
  }

}
