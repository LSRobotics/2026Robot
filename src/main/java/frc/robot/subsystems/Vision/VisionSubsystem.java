package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private final VisionIO io;
    //Field2d field = new Field2d();
    VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Pose3d pose = inputs.pose;
        //field.setRobotPose(pose.toPose2d());
        //SmartDashboard.putData("Field",field);
    
    }
    
}
