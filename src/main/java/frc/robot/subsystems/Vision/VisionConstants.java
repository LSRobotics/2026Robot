package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public final static String camera0name = "Arducam_OV9281_USB_Camera (1)"; //Left
    public final static String camera1name = "Arducam_OV9281_USB_Camera (2)"; //Right
    public final static String camera2name = "Arducam_OV9281_USB_Camera"; //Back

    public final static Transform3d robotToCamera0 = new Transform3d(
            new Translation3d(Inches.of(2.325), Inches.of(14), Inches.of(19.5)),
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90))); // Left Camera
    public final static Transform3d robotToCamera1 = new Transform3d(
            new Translation3d(Inches.of(2.325), Inches.of(-14), Inches.of(19.5)),
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-90))); //Right Camera 
    public final static Transform3d robotToCamera2 = new Transform3d(
            new Translation3d(Inches.of(-12.125), Inches.of(5), Inches.of(18)),
            new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180))); //Back Camera

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(8, 8, Integer.MAX_VALUE); //TODO: REMOVE
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1.5, 1.5, Integer.MAX_VALUE); //TODO: REMOVE

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.25;
    public static double maxZError = 0.1;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.75; // Meters
    public static double angularStdDevBaseline = 1.5; // Radians


    // Standard deviation multipliers for each camera
    public static double[] cameraStdDevFactors = new double[] {
            1.3, // Camera 0
            1.3, // Camera 1
            1.3 // Camera 2
    };

    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}