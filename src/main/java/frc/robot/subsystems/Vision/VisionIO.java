package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs{
        public Pose3d pose = new Pose3d();
        public double latency = -1d;
        //public ArrayList<PhotonTrackedTarget> targets = new ArrayList<>();
    }

    public default void updateInputs(VisionIOInputs inputs){}

}

