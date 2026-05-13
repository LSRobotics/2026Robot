package frc.robot.subsystems.spindexer;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.spindexer.SpindexerIO;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

public class SpindexerMechanism extends Mechanism {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(SpindexerConstants.spindexerRampRatePositive, SpindexerConstants.spindexerRampRateNegative, 0d);

    public SpindexerMechanism(SpindexerIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
            Logger.processInputs("Spindexer", inputs);
        });
    }

    public void runSpindexer(double speed) {
        if (speed==0){
            io.setSpindexerSpeed(0);
        }
        else{
            double newSpeed = speedLimiter.calculate(speed);
            io.setSpindexerSpeed(newSpeed);
        }
    }

    public void runSpindexer(DoubleSupplier speed){
        if (speed.getAsDouble()==0){
            io.setSpindexerSpeed(0);
        }
        else{
            double newSpeed = speedLimiter.calculate(speed.getAsDouble());
            io.setSpindexerSpeed(newSpeed);
        }
    }

    public void runSpindexer(Supplier<Double> speed){
        if (speed.get()==0){
            io.setSpindexerSpeed(0);
        }
        else{
            double newSpeed = speedLimiter.calculate(speed.get());
            io.setSpindexerSpeed(newSpeed);
        }
    }


    public AngularVelocity getSpindexerSpeed() {
        return inputs.spindexerSpeed;
    }

    public Current getSpindexerCurrent() {
        return inputs.spindexerMotorCurrent;
    }

    public Command runSpindexerCommand(double speed) {
        return run(co -> {
            boolean isJammed = false;
            Timer jamTimer = new Timer();

            while (true) {
                if (!isJammed && this.getSpindexerCurrent().gte(SpindexerConstants.spindexerJamThreshold)) {
                    isJammed = true;
                    jamTimer.restart();
                }

                if (isJammed && jamTimer.hasElapsed(SpindexerConstants.jamRecoveryTime)) {
                    isJammed = false;
                    jamTimer.reset();
                }

                if (!isJammed) {
                    this.runSpindexer(speed);
                } else {
                    this.runSpindexer(SpindexerConstants.jamRecoverySpeed);
                }

                Logger.recordOutput("Spindexer/Jammed", isJammed);
                co.yield();
            }
        }).named("Run Spindexer Command");
    }
}
