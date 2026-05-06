package frc.robot.subsystems.spindexer;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.spindexer.SpindexerIO;
import org.wpilib.math.filter.SlewRateLimiter;
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
}