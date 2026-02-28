package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.VisionConstants;
import static frc.robot.subsystems.Vision.VisionConstants.kMultiTagStdDevs;
import static frc.robot.subsystems.Vision.VisionConstants.kSingleTagStdDevs;

public class VisionIOPhoton implements VisionIO {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    public VisionIOPhoton(String cameraName, EstimateConsumer estConsumer, Transform3d cameraToRobot) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera(cameraName);
        estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                cameraToRobot);
    }

    public void updateInputs(VisionIOInputs inputs) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            // Pose3d[] tags = new Pose3d[result.targets.size()]; 
            // Pose3d[] toTags = new Pose3d[result.targets.size()]; 
            // AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            // for (int i=0;i<result.targets.size();i++){
            //     tags[i]=field.getTagPose(result.targets.get(i).fiducialId).get();
            //     toTags[i] = mresult.targets.get(i).bestCameraToTarget; 
            // }
            // Logger.recordOutput("Tags", tags);
            visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(result);
                if (visionEst.isPresent()){Logger.recordOutput(camera.getName()+"/Estpose",visionEst.get().estimatedPose);}
                return;
            }
            Logger.recordOutput(camera.getName()+"/Estpose",visionEst.get().estimatedPose);

            updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        var estStdDevs = getEstimationStdDevs();

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
            if (visionEst.isPresent()) {
                inputs.pose = visionEst.get().estimatedPose;
            }
        }

    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {

            curStdDevs = kSingleTagStdDevs;

        } else {

            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {

                curStdDevs = kSingleTagStdDevs;
            } else {

                avgDist /= numTags;

                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;

                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

}
