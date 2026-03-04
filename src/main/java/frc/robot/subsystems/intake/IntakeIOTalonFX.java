package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> intakeSpeed = intakeMotor.getRotorVelocity();

    public IntakeIOTalonFX() {
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true));

        intakeCurrent.setUpdateFrequency(Hertz.of(50));
        intakeSpeed.setUpdateFrequency(Hertz.of(50));
        intakeMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeCurrent.refresh();
        intakeSpeed.refresh();
        inputs.intakeRollerMotorCurrent = intakeCurrent.getValue();
        inputs.intakeRollerSpeed = intakeSpeed.getValue();
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