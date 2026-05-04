package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import frc.robot.subsystems.arm.ArmIO;

public class ArmSubsystem extends SubsystemBase {
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private ArmLimitSwitchIO limitSwitchIO;
    private ArmLimitSwitchIOInputsAutoLogged limitSwitchInputs = new ArmLimitSwitchIOInputsAutoLogged();

    public ArmSubsystem(ArmIO armIO, ArmLimitSwitchIO limitSwitchIO) {
        this.armIO = armIO;
        this.limitSwitchIO = limitSwitchIO;
    }

    public void runArm(double speed) {
        armIO.setArmSpeed(speed);
    }

    public void setAngle(Angle angle) {
        armIO.setArmAngle(angle);
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        limitSwitchIO.updateInputs(limitSwitchInputs);
        Logger.processInputs("Arm", armInputs);
        if (armInputs.armAngle.isNear(ArmConstants.ArmMotorConstants.ARM_DEPLOY_ANGLE, ArmConstants.ArmMotorConstants.ARM_TOLERANCE)) {
            Logger.recordOutput("Arm/AtDeployAngle", true);
        } else {
            Logger.recordOutput("Arm/AtDeployAngle", false);
        }
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
