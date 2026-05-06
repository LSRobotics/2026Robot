package frc.robot.subsystems.arm;

import java.util.concurrent.ScheduledExecutorService;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import frc.robot.subsystems.arm.ArmIO;

public class ArmMechanism extends Mechanism {
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private ArmLimitSwitchIO limitSwitchIO;
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
