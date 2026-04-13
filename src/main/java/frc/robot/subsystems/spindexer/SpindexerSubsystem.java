package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.spindexer.SpindexerIO;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class SpindexerSubsystem extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(SpindexerConstants.spindexerRampRatePositive, SpindexerConstants.spindexerRampRateNegative, 0d);

    public SpindexerSubsystem(SpindexerIO io) {
        this.io = io;
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
    }

    public AngularVelocity getSpindexerSpeed() {
        return inputs.spindexerSpeed;
    }

    public Current getSpindexerCurrent() {
        return inputs.spindexerMotorCurrent;
    }
}