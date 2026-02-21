package frc.robot.subsystems.Shooter;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterFlywheelIO;

public class ShooterFlywheelIOTalonFX implements ShooterFlywheelIO {
    private final TalonFX flywheelMotor1 = new TalonFX(ShooterConstants.FlywheelConstants.flywheelMotor1ID);
    private final TalonFX flywheelMotor2 = new TalonFX(ShooterConstants.FlywheelConstants.flywheelMotor2ID);

    public ShooterFlywheelIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(ShooterConstants.FlywheelConstants.statorCurrentLimit);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelMotor1.getConfigurator().apply(config);
        flywheelMotor2.getConfigurator().apply(config);

        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        flywheelMotor1.setControl(new VelocityDutyCycle(velocity));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        flywheelMotor1.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setSpeed(double speed) {
        flywheelMotor1.set(speed);
    }    

    @Override
    public void updateInputs(ShooterFlywheelIOInputs inputs) {
        inputs.velocity = flywheelMotor1.getVelocity().getValue();
        inputs.motor1Current = flywheelMotor1.getStatorCurrent().getValue();
        inputs.motor2Current = flywheelMotor2.getStatorCurrent().getValue();
    }

}
