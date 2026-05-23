package frc.robot.subsystems.Turret;

import static org.wpilib.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStation;

import frc.robot.Robot;
import frc.robot.util.MathUtils;

public class TurretMechanism extends Mechanism{

    private final TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged(); 

    public TurretMechanism(TurretIO io){
        this.io = io;
        io.zeroEncoder();
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
            Logger.processInputs("Turret", inputs);
        });
    }

    public void setVoltage(Voltage voltage){
        io.setTurretVoltage(voltage);
    }

    public void setSpeed(double speed){
        io.setTurretSpeed(speed);
    }

    public Command runTurretCommand(DoubleSupplier speed) {
        return run(co -> {
            while (true) {
                if (Robot.isDisabled()) {
                    break;
                }

                double clampedSpeed = MathUtils.clamp(-TurretConstants.maxOpSpeed, TurretConstants.maxOpSpeed, speed.getAsDouble());

                if (getAngle().lte(TurretConstants.turretRangeOneWay.unaryMinus()) && clampedSpeed < 0) {
                    clampedSpeed = 0;
                } else if (getAngle().gte(TurretConstants.turretRangeOneWay) && clampedSpeed > 0) {
                    clampedSpeed = 0;
                }

                setSpeed(clampedSpeed);
                co.yield();
            }

            setSpeed(0);
        }).whenCanceled(() -> setSpeed(0)).named("Run Turret Command");
    }

    public Command runTurretCommand(double speed) {
        return runTurretCommand(() -> speed);
    }

    public Command runTurretToAngleCommand(Angle angle) {
        return runTurretToAngleCommand(() -> angle);
    }

    public Command runTurretToAngleCommand(Supplier<Angle> angle) {
        return run(co -> {
            PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
            pid.setTolerance(TurretConstants.tolerance);

            while (true) {
                if (Robot.isDisabled()) {
                    break;
                }

                double setpoint = MathUtils.clamp(
                        angle.get().in(Degrees),
                        -TurretConstants.turretRangeOneWay.in(Degrees),
                        TurretConstants.turretRangeOneWay.in(Degrees));

                pid.setSetpoint(setpoint);

                double speed = pid.calculate(getAngle().in(Degrees));
                speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);

                setSpeed(speed);
                Logger.recordOutput("Turret/PID_Error", pid.getError());
                Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());
                Logger.recordOutput("Turret/Speed", speed);

                if (pid.atSetpoint()) {
                    break;
                }

                co.yield();
            }

            setSpeed(0);
        }).whenCanceled(() -> setSpeed(0)).named("Run Turret To Angle Command");
    }

    public Angle getAngle(){
        return inputs.turretAngle;
    }

    public void zeroEncoder(){
        io.zeroEncoder();
    }

    
}
