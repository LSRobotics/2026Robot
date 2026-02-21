package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmMotorConstants.ARM_MOTOR_ID);

    public ArmIOTalonFX() {
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ArmMotorConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armMotorCurrent = Amps.of(armMotor.getSupplyCurrent().getValueAsDouble());
        inputs.armAngle = armMotor.getRotorPosition().getValue();
        inputs.armOut = inputs.armAngle.equals(ArmMotorConstants.ARM_DEPLOY_ANGLE);
        inputs.armSpeed = armMotor.getRotorVelocity().getValue();
    }

    @Override
    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        armMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }
    
    // public AngularVelocity getArmSpeed() {
    //     return RPM.of(armMotor.getRotorVelocity().getValueAsDouble());
    // }
    
}
