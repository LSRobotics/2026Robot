package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class IntakeIOMotor implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    public IntakeIOMotor() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeRollerMotorCurrent = Amps.of(intakeMotor.getSupplyCurrent().getValueAsDouble());
        inputs.intakeRollerSpeed = intakeMotor.getRotorVelocity().getValue();
    }

    @Override
    public void setRollerSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    public AngularVelocity getRollerSpeed() {
        return RPM.of(intakeMotor.getRotorVelocity().getValueAsDouble());
    }
}