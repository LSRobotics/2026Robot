package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmConstants.ArmMotorConstants;;

public class ArmIOSparkMax implements ArmIO {
    private final SparkMax armMotor = new SparkMax(ArmMotorConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    private final SparkBaseConfig leaderConfig = new SparkMaxConfig().smartCurrentLimit((int) ArmMotorConstants.STATOR_CURRENT_LIMIT.in(Amps)).secondaryCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT.in(Amps)).idleMode(IdleMode.kBrake);
    private final SparkMax armMotorFollower = new SparkMax(ArmMotorConstants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
    private final SparkBaseConfig followerConfig = new SparkMaxConfig().smartCurrentLimit((int) ArmMotorConstants.STATOR_CURRENT_LIMIT.in(Amps)).secondaryCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT.in(Amps)).idleMode(IdleMode.kBrake);


    public ArmIOSparkMax() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit((int) ArmMotorConstants.STATOR_CURRENT_LIMIT.in(Amps))
            .secondaryCurrentLimit(ArmMotorConstants.SUPPLY_CURRENT_LIMIT.in(Amps))
            .idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(ArmMotorConstants.ARM_MOTOR_ID);

        armMotorFollower.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armMotorCurrent = Amps.of(armMotor.getOutputCurrent());
        inputs.armAngle = Angle.ofBaseUnits(armMotor.getEncoder().getPosition(), Degrees);
        inputs.armOut = inputs.armAngle.equals(ArmMotorConstants.ARM_DEPLOY_ANGLE);
        inputs.armSpeed = AngularVelocity.ofBaseUnits(armMotor.getEncoder().getVelocity(), RPM);
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
    public void setArmAngle(Angle angle) {
        armMotor.getEncoder().setPosition(angle.in(Degrees));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    @Override 
    public void setBrakeOnNeutral(boolean brake){
        if (brake){
            armMotor.configure(leaderConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            armMotorFollower.configure(followerConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        else{
            armMotor.configure(leaderConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            armMotorFollower.configure(followerConfig.idleMode(IdleMode.kBrake),ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
    
    // public AngularVelocity getArmSpeed() {
    //     return RPM.of(armMotor.getRotorVelocity().getValueAsDouble());
    // }
    
}
