package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
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
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TempApriltagVision extends SubsystemBase {
    
    private PhotonCamera mBackRightCam, mBackLeftCam, mFrontRightCam, mFrontLeftCam;
    private PhotonPoseEstimator mBackRightEstimator, mBackLeftEstimator, mFrontRightEstimator, mFrontLeftEstimator;
    private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPipelineResult mBackRightAprilTagResult, mBackLeftAprilTagResult, mFrontRightAprilTagResult, mFrontLeftAprilTagResult;
    private Optional<EstimatedRobotPose> mBackRight, mBackLeft, mFrontLeft, mFrontRight;
    @SuppressWarnings("unchecked")
    private final StructArrayPublisher<AprilTag> mApriltagPublisherBR, mApriltagPublisherBL, mApriltagPublisherFL, mApriltagPublisherFR;
    // Array of subscribers for camera subtables
    private final RawSubscriber rawBytesSubscribersBR, rawBytesSubscribersBL, rawBytesSubscribersFL, rawBytesSubscribersFR;
    // Array of subscriber wait handles for camera subtables, used to lookup camera index from wait handle
    private final int waitHandlesBR, waitHandlesBL, waitHandlesFL, waitHandlesFR;
    private final Packet packet = new Packet(1);

    public TempApriltagVision(){
        PhotonCamera.setVersionCheckEnabled(false);

        mBackRightCam = new PhotonCamera(Constants.Vision.aprilTagBackRight.camera);
        mBackLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);
        mFrontRightCam = new PhotonCamera(Constants.Vision.aprilTagBackRight.camera);
        mFrontLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);

        mBackRightCam.setDriverMode(false);
        mBackLeftCam.setDriverMode(false);
        mFrontRightCam.setDriverMode(false);
        mFrontLeftCam.setDriverMode(false);

        mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackRightCam, Constants.Vision.aprilTagBackRight.robotToCamera);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.Vision.aprilTagBackLeft.robotToCamera);
        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.Vision.aprilTagFrontRight.robotToCamera);
        mFrontLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontLeftCam, Constants.Vision.aprilTagFrontLeft.robotToCamera);

        mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        mFrontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        mBackRightEstimator.setTagModel(TargetModel.kAprilTag36h11);
        mBackLeftEstimator.setTagModel(TargetModel.kAprilTag36h11);
        mFrontRightEstimator.setTagModel(TargetModel.kAprilTag36h11);
        mFrontLeftEstimator.setTagModel(TargetModel.kAprilTag36h11);

        mBackRightEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackRight.robotToCamera);
        mBackLeftEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackLeft.robotToCamera);
        mFrontRightEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagFrontRight.robotToCamera);
        mFrontLeftEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagFrontLeft.robotToCamera);

        mApriltagPublisherBR = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags-" + Constants.Vision.aprilTagBackRight.camera, new AprilTagStruct()).publish();
        mApriltagPublisherBL = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags-" + Constants.Vision.aprilTagBackLeft.camera, new AprilTagStruct()).publish();
        mApriltagPublisherFR = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags-" + Constants.Vision.aprilTagFrontRight.camera, new AprilTagStruct()).publish();
        mApriltagPublisherFL = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags-" + Constants.Vision.aprilTagFrontLeft.camera, new AprilTagStruct()).publish();

        rawBytesSubscribersBR = NetworkTableInstance.getDefault().getTable("photonvision")
                                .getSubTable(Constants.Vision.aprilTagBackRight.camera).getRawTopic("rawBytes")
                                .subscribe("rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));

        rawBytesSubscribersBL = NetworkTableInstance.getDefault().getTable("photonvision")
                                .getSubTable(Constants.Vision.aprilTagBackLeft.camera).getRawTopic("rawBytes")
                                .subscribe("rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));

        rawBytesSubscribersFR = NetworkTableInstance.getDefault().getTable("photonvision")
                                .getSubTable(Constants.Vision.aprilTagFrontRight.camera).getRawTopic("rawBytes")
                                .subscribe("rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));

        rawBytesSubscribersFL = NetworkTableInstance.getDefault().getTable("photonvision")
                                .getSubTable(Constants.Vision.aprilTagFrontLeft.camera).getRawTopic("rawBytes")
                                .subscribe("rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));  
                                
        waitHandlesBR = rawBytesSubscribersBR.getHandle();
        waitHandlesBL = rawBytesSubscribersBL.getHandle();
        waitHandlesFR = rawBytesSubscribersFR.getHandle();
        waitHandlesFL = rawBytesSubscribersFL.getHandle();

        
    }

    @Override
    public void periodic() {
        if (Constants.Vision.visionTurnedOn){
            if (this.mBackRightCam != null){
                mBackRightAprilTagResult = mBackRightCam.getLatestResult();

                mBackRight = getBackRightEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mBackRightAprilTagResult);

                if (mBackRight.isPresent() && seesMoreThanOneTag()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(addCameraTransform(mBackRight.get().estimatedPose.toPose2d(), Constants.Vision.aprilTagBackRight.robotToCamera).getTranslation()
                        , Swerve.poseEstimator.getEstimatedPosition().getRotation()), mBackRightAprilTagResult.getTimestampSeconds(), getEstimationStdDevs(Swerve.poseEstimator.getEstimatedPosition(), mBackRightAprilTagResult));
                }

                // Send the AprilTag(s) to NT for AdvantageScope
                mApriltagPublisherBR.accept(mBackRightAprilTagResult.targets.stream().map(target ->
                    getTargetPose(target, Swerve.poseEstimator.getEstimatedPosition(), mBackRightEstimator.getRobotToCameraTransform())
                ).toArray(AprilTag[]::new));
            }

            if (this.mBackLeftCam != null){
                mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();

                mBackLeft = getBackLeftEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mBackLeftAprilTagResult);

                
                if (mBackLeft.isPresent() && seesMoreThanOneTag()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(addCameraTransform(mBackLeft.get().estimatedPose.toPose2d(), Constants.Vision.aprilTagBackLeft.robotToCamera).getTranslation()
                        , Swerve.poseEstimator.getEstimatedPosition().getRotation()), mBackLeftAprilTagResult.getTimestampSeconds(), getEstimationStdDevs(Swerve.poseEstimator.getEstimatedPosition(), mBackLeftAprilTagResult));
                }

                // Send the AprilTag(s) to NT for AdvantageScope
                mApriltagPublisherBL.accept(mBackLeftAprilTagResult.targets.stream().map(target ->
                    getTargetPose(target, Swerve.poseEstimator.getEstimatedPosition(), mBackLeftEstimator.getRobotToCameraTransform())
                ).toArray(AprilTag[]::new));
            }

            if (this.mFrontRightCam != null){
                mFrontRightAprilTagResult = mFrontRightCam.getLatestResult();

                mFrontRight = getFrontRightEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mFrontRightAprilTagResult);

                if (mFrontRight.isPresent() && seesMoreThanOneTag()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(addCameraTransform(mFrontRight.get().estimatedPose.toPose2d(), Constants.Vision.aprilTagFrontRight.robotToCamera).getTranslation()
                        , Swerve.poseEstimator.getEstimatedPosition().getRotation()), mFrontRightAprilTagResult.getTimestampSeconds(), getEstimationStdDevs(Swerve.poseEstimator.getEstimatedPosition(), mFrontRightAprilTagResult));
                }

                // Send the AprilTag(s) to NT for AdvantageScope
                mApriltagPublisherFR.accept(mFrontRightAprilTagResult.targets.stream().map(target ->
                    getTargetPose(target, Swerve.poseEstimator.getEstimatedPosition(), mFrontRightEstimator.getRobotToCameraTransform())
                ).toArray(AprilTag[]::new));
            }

            if (this.mFrontLeft != null){
                mFrontLeftAprilTagResult = mFrontLeftCam.getLatestResult();

                mFrontLeft = getFrontLeftEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mFrontLeftAprilTagResult);

                if (mFrontLeft.isPresent() && seesMoreThanOneTag()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(addCameraTransform(mFrontLeft.get().estimatedPose.toPose2d(), Constants.Vision.aprilTagFrontLeft.robotToCamera).getTranslation()
                        , Swerve.poseEstimator.getEstimatedPosition().getRotation()), mFrontLeftAprilTagResult.getTimestampSeconds(), getEstimationStdDevs(Swerve.poseEstimator.getEstimatedPosition(), mFrontLeftAprilTagResult));
                }

                // Send the AprilTag(s) to NT for AdvantageScope
                mApriltagPublisherFL.accept(mFrontLeftAprilTagResult.targets.stream().map(target ->
                    getTargetPose(target, Swerve.poseEstimator.getEstimatedPosition(), mFrontLeftEstimator.getRobotToCameraTransform())
                ).toArray(AprilTag[]::new));
            }


            // mApriltagPublisherBR.close();
            // mApriltagPublisherBL.close();
            // mApriltagPublisherFR.close();
            // mApriltagPublisherFL.close();

            // rawBytesSubscribersBR.close();
            // rawBytesSubscribersBL.close();
            // rawBytesSubscribersFR.close();
            // rawBytesSubscribersFL.close();

        }


    }

    public boolean seesMoreThanOneTag(){
        // int temp = 0;

        // if (this.mBackRightCam != null){
        //     if (mBackRightAprilTagResult != null){
        //         temp = temp + mBackRightAprilTagResult.getTargets().size();
        //     }
            
        // }

        // if (this.mBackLeftCam != null){
        //     if (mBackLeftAprilTagResult != null){
        //         temp = temp + mBackLeftAprilTagResult.getTargets().size();
        //     }
        // }

        // if (this.mBackRightCam != null){
        //     if (mFrontRightAprilTagResult != null){
        //         temp = temp + mFrontRightAprilTagResult.getTargets().size();
        //     } 
        // }

        // if (this.mBackRightCam != null){
        //     if (mFrontLeftAprilTagResult != null){
        //         temp = temp + mFrontLeftAprilTagResult.getTargets().size();
        //     }
        // }

        // return temp > 1;

        return true;
        
    }

    public Optional<EstimatedRobotPose> getBackRightEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mBackRightAprilTagResult) {
        mBackRightEstimator.setReferencePose(prevEstimatedRobotPose);
        return mBackRightEstimator.update(mBackRightAprilTagResult);
    }

    public Optional<EstimatedRobotPose> getBackLeftEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mBackLeftAprilTagResult) {
        mBackLeftEstimator.setReferencePose(prevEstimatedRobotPose);
        return mBackLeftEstimator.update(mBackLeftAprilTagResult);
    }

    public Optional<EstimatedRobotPose> getFrontLeftEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mFrontLeftAprilTagResult) {
        mFrontLeftEstimator.setReferencePose(prevEstimatedRobotPose);
        return mFrontLeftEstimator.update(mFrontLeftAprilTagResult);
    }

    public Optional<EstimatedRobotPose> getFrontRightEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mFrontRightAprilTagResult) {
        mFrontRightEstimator.setReferencePose(prevEstimatedRobotPose);
        return mFrontRightEstimator.update(mFrontRightAprilTagResult);
    }

    public Pose2d addCameraTransform(Pose2d a, Transform3d b){
        return new Pose3d(a).transformBy(b).toPose2d();
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    //FT --> BR
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult a) {
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        var targets = a;
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