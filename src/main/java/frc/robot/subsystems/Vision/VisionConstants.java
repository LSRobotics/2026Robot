package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.Matrix;

public class VisionConstants {
     public final static Transform3d cameraToRobot1 = new Transform3d(
        new Translation3d(Inches.of(-13),Inches.of(5.5),Inches.of(19.5)), 
        new Rotation3d(Degrees.of(0),Degrees.of(0),Degrees.of(180))); //Back camera
    public final static Transform3d cameraToRobot2 = new Transform3d(
        new Translation3d(Inches.of(12.5),Inches.of(13),Inches.of(18)), 
        new Rotation3d(Degrees.of(0),Degrees.of(0),Degrees.of(0))); //INtake camera

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(8, 8, Integer.MAX_VALUE);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1.5, 1.5, Integer.MAX_VALUE);
}
