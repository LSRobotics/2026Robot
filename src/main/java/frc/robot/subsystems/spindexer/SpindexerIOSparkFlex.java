package frc.robot.subsystems.spindexer;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.spindexer.SpindexerIO;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class SpindexerIOSparkFlex implements SpindexerIO {
    private final SparkFlex spindexerMotor = new SparkFlex(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

    public SpindexerIOSparkFlex(){}

    @Override
    public void updateInputs(SpindexerIOInputs inputs){
        inputs.spindexerMotorCurrent = Amps.of(spindexerMotor.getOutputCurrent());
        inputs.spindexerSpeed = AngularVelocity.ofBaseUnits(spindexerMotor.getAppliedOutput(), RPM);
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
