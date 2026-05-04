package frc.robot.subsystems.Turret;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotation;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.units.measure.Voltage;
import org.wpilib.smartdashboard.SmartDashboard;

public class TurretIOTalon implements TurretIO {
    private TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID);
    public TurretIOTalon(){
        turretMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive));
        turretMotor.setPosition(Degrees.of(0));
    }
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorAngle = turretMotor.getPosition().getValue();
        inputs.turretVelocity = turretMotor.getVelocity().getValue();
        inputs.turretCurrent = turretMotor.getSupplyCurrent().getValue();
        inputs.turretAngle = inputs.motorAngle.times(TurretConstants.turretGearRatio); 
        inputs.turretDegrees = inputs.turretAngle.in(Degrees);
    }

    @Override
    public void setTurretVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }

    @Override
    public void setTurretSpeed(double speed) {
        turretMotor.set(speed);
    }

    @Override
    public void zeroEncoder(){
        turretMotor.setPosition(0);
    }
    
}
