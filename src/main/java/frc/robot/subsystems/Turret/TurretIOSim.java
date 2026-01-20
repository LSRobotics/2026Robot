package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretIOSim implements TurretIO {


    private static final double mechWidth = 1.0;
    private static final double mechHeight = 1.0;
    private static final double turretLength = 0.5;

    private static final double visualOffsetDeg = 90.0;

    private final LoggedMechanism2d mech =
            new LoggedMechanism2d(mechWidth, mechHeight);

    private final LoggedMechanismRoot2d root =
            mech.getRoot(
                    "TurretRoot",
                    (mechWidth / 2.0) + TurretConstants.turretOffset.getX(),
                    (mechHeight / 2.0) + TurretConstants.turretOffset.getY()
            );

    private final LoggedMechanismLigament2d turret =
            root.append(
                    new LoggedMechanismLigament2d(
                            "Turret",
                            turretLength,
                            visualOffsetDeg
                    )
            );

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAngle = Degrees.of(turret.getAngle() - visualOffsetDeg);

        Logger.recordOutput("Turret/Mechanism", mech);
    }

    @Override
    public void setTurretVoltage(Voltage voltage) {
    }

    @Override
    public void setTurretSpeed(double speed) {
        double deltaDeg =
                speed
                * TurretConstants.maxSpeed.in(DegreesPerSecond)
                * TurretConstants.turretGearRatio
                * (1d / 50d); 

        turret.setAngle(turret.getAngle() + deltaDeg);
        turret.setColor(new Color8Bit(0,255,0));
    }

    @Override
    public void resetTurretEncoder(Angle angle) {
    }
}
