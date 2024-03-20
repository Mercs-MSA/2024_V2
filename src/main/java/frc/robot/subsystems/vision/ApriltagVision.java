package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ApriltagVision extends SubsystemBase {
    
    private PhotonCamera mBackRightCam, mBackLeftCam;
    private PhotonPoseEstimator mBackRightEstimator, mBackLeftEstimator;
    private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPipelineResult mBackRightAprilTagResult, mBackLeftAprilTagResult;
    private Optional<EstimatedRobotPose> mBackRight, mBackLeft;
    @SuppressWarnings("unchecked")
    private final StructArrayPublisher<AprilTag> mApriltagPublisherBR, mApriltagPublisherBL;

    public ApriltagVision(){
        PhotonCamera.setVersionCheckEnabled(false);

        mBackRightCam = new PhotonCamera(Constants.Vision.aprilTagBackRight.camera);
        mBackLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);

        mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackRightCam, Constants.Vision.aprilTagBackRight.robotToCamera);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.Vision.aprilTagBackLeft.robotToCamera);

        mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mBackRightEstimator.setTagModel(TargetModel.kAprilTag36h11);
        mBackLeftEstimator.setTagModel(TargetModel.kAprilTag36h11);

        mBackRightEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackRight.robotToCamera);
        mBackLeftEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackLeft.robotToCamera);

        mApriltagPublisherBR = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags-" + Constants.Vision.aprilTagBackRight.camera, new AprilTagStruct()).publish();
        mApriltagPublisherBL = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags-" + Constants.Vision.aprilTagBackLeft.camera, new AprilTagStruct()).publish();
        
    }

    @Override
    public void periodic() {
        if (Constants.Vision.visionTurnedOn){
            if (this.mBackRightCam != null){
                mBackRightAprilTagResult = mBackRightCam.getLatestResult();

                mBackRight = getBackRightEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mBackRightAprilTagResult);

                if (mBackRight.isPresent()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(mBackRight.get().estimatedPose.toPose2d().getTranslation(), Swerve.poseEstimator.getEstimatedPosition().getRotation()), mBackRightAprilTagResult.getTimestampSeconds(), getBREstimationStdDevs(Swerve.poseEstimator.getEstimatedPosition()));
                }

                // Send the AprilTag(s) to NT for AdvantageScope
                mApriltagPublisherBR.accept(mBackRightAprilTagResult.targets.stream().map(target ->
                    getTargetPose(target, Swerve.poseEstimator.getEstimatedPosition(), mBackRightEstimator.getRobotToCameraTransform())
                ).toArray(AprilTag[]::new));
            }

            if (this.mBackLeftCam != null){
                mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();

                mBackLeft = getBackLeftEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mBackLeftAprilTagResult);

                if (mBackLeft.isPresent()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(mBackLeft.get().estimatedPose.toPose2d().getTranslation(), Swerve.poseEstimator.getEstimatedPosition().getRotation()), mBackLeftAprilTagResult.getTimestampSeconds(), getBLEstimationStdDevs(Swerve.poseEstimator.getEstimatedPosition()));
                }

                // Send the AprilTag(s) to NT for AdvantageScope
                mApriltagPublisherBL.accept(mBackLeftAprilTagResult.targets.stream().map(target ->
                    getTargetPose(target, Swerve.poseEstimator.getEstimatedPosition(), mBackLeftEstimator.getRobotToCameraTransform())
                ).toArray(AprilTag[]::new));

            }

            // SmartDashboard.putData("mFrontRightAprilTagResult", mFrontRightAprilTagResult.getLatestResult().targets);
            mApriltagPublisherBR.close();
            mApriltagPublisherBL.close();

        }


    }

    public Optional<EstimatedRobotPose> getBackRightEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mFrontRightAprilTagResult) {
        mBackRightEstimator.setReferencePose(prevEstimatedRobotPose);
        return mBackRightEstimator.update(mFrontRightAprilTagResult);
    }

    public Optional<EstimatedRobotPose> getBackLeftEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mBackLeftAprilTagResult) {
        mBackLeftEstimator.setReferencePose(prevEstimatedRobotPose);
        return mBackLeftEstimator.update(mBackLeftAprilTagResult);
    }

       /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    //FT --> BR
    public Matrix<N3, N1> getBREstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = mBackRightAprilTagResult;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets.targets) {
            var tagPose = mBackRightEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }


    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getBLEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = mBackLeftAprilTagResult;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets.targets) {
            var tagPose = mBackRightEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * Transform a target from PhotonVision to a pose on the field
     * @param target target data from PhotonVision
     * @param robotPose current pose of the robot
     * @param robotToCamera transform from robot to the camera that saw the target
     * @return an AprilTag with an ID and pose
     */
    private static AprilTag getTargetPose(PhotonTrackedTarget target, Pose2d robotPose, Transform3d robotToCamera) {
        var targetPose = new Pose3d(robotPose)
            .transformBy(robotToCamera)
            .transformBy(target.getBestCameraToTarget());
        return new AprilTag(target.getFiducialId(), targetPose);
    }

    
}