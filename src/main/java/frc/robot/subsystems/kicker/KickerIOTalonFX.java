package frc.robot.subsystems.kicker;



import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class KickerIOTalonFX implements KickerIO{
     private final TalonFX kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_ID);

    public KickerIOTalonFX() {}

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.kickerMotorCurrent = Amps.of(kickerMotor.getSupplyCurrent().getValueAsDouble());
        inputs.kickerSpeed = kickerMotor.getRotorVelocity().getValue();
    }

    @Override
    public void setKickerSpeed(double speed) {
        kickerMotor.set(speed);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        kickerMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        kickerMotor.stopMotor();
    }

    public AngularVelocity getKickerSpeed() {
        return RPM.of(kickerMotor.getRotorVelocity().getValueAsDouble());
    }
}
