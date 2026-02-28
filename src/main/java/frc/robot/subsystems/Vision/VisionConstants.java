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
        new Translation3d(Inches.of(1.5),Inches.of(6),Inches.of(18.5)), 
        new Rotation3d(0,0,0));
    public final static Transform3d cameraToRobot2 = new Transform3d(
        new Translation3d(Inches.of(-1.5),Inches.of(-3.25),Inches.of(18.5)), 
        new Rotation3d(Degrees.of(0),Degrees.of(0),Degrees.of(180)));

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(9, 9, 25);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1, 1, 2.5);
}
