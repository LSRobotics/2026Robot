package frc.robot.subsystems.spindexer;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.spindexer.SpindexerIO;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static org.wpilib.units.Units.Amps;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.units.measure.AngularVelocity;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Volts;

public class SpindexerIOTalonFX implements SpindexerIO {
    private final TalonFX spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID);

    public SpindexerIOTalonFX(){}

    @Override
    public void updateInputs(SpindexerIOInputs inputs){
        inputs.spindexerMotorCurrent = spindexerMotor.getStatorCurrent().getValue();
        inputs.spindexerSpeed = spindexerMotor.getVelocity().getValue();
    }

    @Override
    public void setSpindexerSpeed(double speed) {
        spindexerMotor.set(speed);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        spindexerMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        spindexerMotor.stopMotor();
    }

    // public AngularVelocity getRollerSpeed() {
    //     return RPM.of(spindexerMotor.getRotorVelocity().getValueAsDouble());
    // }
}
