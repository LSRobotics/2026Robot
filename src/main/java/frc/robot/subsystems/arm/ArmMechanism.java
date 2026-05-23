package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;

public class ArmMechanism extends Mechanism {
    private final ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private final ArmLimitSwitchIO limitSwitchIO;
    private ArmLimitSwitchIOInputsAutoLogged limitSwitchInputs = new ArmLimitSwitchIOInputsAutoLogged();

    public ArmMechanism(ArmIO armIO, ArmLimitSwitchIO limitSwitchIO) {
        this.armIO = armIO;
        this.limitSwitchIO = limitSwitchIO;
        Scheduler.getDefault().addPeriodic(() -> {
            armIO.updateInputs(armInputs);
            limitSwitchIO.updateInputs(limitSwitchInputs);
            Logger.processInputs("Arm", armInputs);
        });
    }

    public void runArm(double speed) {
        armIO.setArmSpeed(speed);
    }

    public void setAngle(Angle angle) {
        armIO.setArmAngle(angle);
    }

    public Command runArmCommand(Angle angle) {
        return runArmCommand(() -> angle, () -> ArmConstants.ArmMotorConstants.ARM_SPEED);
    }

    public Command runArmCommand(Angle angle, double speed) {
        return runArmCommand(() -> angle, () -> speed);
    }

    public Command runArmCommand(Supplier<Angle> angle, DoubleSupplier speed) {
        return run(co -> {
            while (true) {
                Angle targetAngle = angle.get();
                double targetArmDegrees = targetAngle.in(Degrees);
                double motorDegrees = getArmEncoder().in(Degrees);
                double armDegrees = motorDegrees / ArmMotorConstants.gearRatio;
                double armErrorDegrees = targetArmDegrees - armDegrees;
                double targetMotorDegrees = targetArmDegrees * ArmMotorConstants.gearRatio;

                Logger.recordOutput("Arm/TargetArmDegrees", targetArmDegrees);
                Logger.recordOutput("Arm/TargetMotorDegrees", targetMotorDegrees);
                Logger.recordOutput("Arm/MotorAngleDegrees", motorDegrees);
                Logger.recordOutput("Arm/ArmAngleDegrees", armDegrees);
                Logger.recordOutput("Arm/ArmErrorDegrees", armErrorDegrees);

                if (Math.abs(armErrorDegrees) <= ArmConstants.ArmMotorConstants.ARM_TOLERANCE) {
                    runArm(0);
                    break;
                }

                double motorErrorDegrees = targetMotorDegrees - motorDegrees;
                double direction = Math.signum(motorErrorDegrees);

                if (armDegrees > ArmConstants.ArmMotorConstants.ARM_REST_ANGLE.in(Degrees) && direction == 1d) {
                    direction = 0;
                } else if (armDegrees < ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE.in(Degrees) && direction == -1d) {
                    direction = 0;
                }

                runArm(direction * speed.getAsDouble());
                co.yield();
            }

            runArm(0);
        }).whenCanceled(() -> runArm(0)).named("Run Arm Command");
    }

    public AngularVelocity getArmSpeed() {
        return armInputs.armSpeed;
    }

    public Angle getArmEncoder() {
        return armInputs.armAngle;
    }

    public boolean getLimitSwitch() {
        return limitSwitchInputs.activity;
    }
}
