package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KickerIOTalonFX implements KickerIO {
    private final TalonFX kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_ID);

    public KickerIOTalonFX() {
        kickerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.kickerVelocity = RotationsPerSecond.of(kickerMotor.getVelocity().getValueAsDouble());
        inputs.kickerSpeed = kickerMotor.get();
    }

    @Override
    public void setKickerSpeed(double speed) {
        kickerMotor.set(speed);
    }

    @Override
    public void stop() {
        kickerMotor.set(0);
    }
}