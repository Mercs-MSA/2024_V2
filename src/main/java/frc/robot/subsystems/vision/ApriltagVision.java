package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ApriltagVision extends SubsystemBase {
    
    private PhotonCamera mFrontRightCam, mBackLeftCam;
    private PhotonPoseEstimator mFrontRightEstimator, mBackLeftEstimator;
    private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPipelineResult mFrontRightAprilTagResult, mBackLeftAprilTagResult;
    private Optional<EstimatedRobotPose> mFrontRight, mBackLeft;

    public ApriltagVision(){
        PhotonCamera.setVersionCheckEnabled(false);

        mFrontRightCam = new PhotonCamera(Constants.Vision.aprilTagFrontRight.camera);
        mBackLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);

        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.Vision.aprilTagFrontRight.robotToCamera);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.Vision.aprilTagBackLeft.robotToCamera);

        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mFrontRightEstimator.setTagModel(TargetModel.kAprilTag36h11);
        mBackLeftEstimator.setTagModel(TargetModel.kAprilTag36h11);

        mFrontRightEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagFrontRight.robotToCamera);
        mBackLeftEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackLeft.robotToCamera);

        
    }

    @Override
    public void periodic() {
        if (Constants.Vision.visionTurnedOn){
            if (this.mFrontRightCam != null){
                mFrontRightAprilTagResult = mFrontRightCam.getLatestResult();

                mFrontRight = getBackLeftEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mFrontRightAprilTagResult);

                if (mFrontRight.isPresent()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(mFrontRight.get().estimatedPose.toPose2d().getTranslation(), Swerve.poseEstimator.getEstimatedPosition().getRotation()), mFrontRightAprilTagResult.getTimestampSeconds());
                }

            }

            if (this.mBackLeftCam != null){
                mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();

                mBackLeft = getFrontRightEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mBackLeftAprilTagResult);

                if (mBackLeft.isPresent()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(mBackLeft.get().estimatedPose.toPose2d().getTranslation(), Swerve.poseEstimator.getEstimatedPosition().getRotation()), mBackLeftAprilTagResult.getTimestampSeconds());
                }


            }

        }
    }

    public Optional<EstimatedRobotPose> getFrontRightEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mFrontRightAprilTagResult) {
        mFrontRightEstimator.setReferencePose(prevEstimatedRobotPose);
        return mFrontRightEstimator.update(mFrontRightAprilTagResult);
    }

    public Optional<EstimatedRobotPose> getBackLeftEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mBackLeftAprilTagResult) {
        mBackLeftEstimator.setReferencePose(prevEstimatedRobotPose);
        return mBackLeftEstimator.update(mBackLeftAprilTagResult);
    }
    
}


class AprilTagStruct implements Struct<AprilTag> {
  @Override
  public Class<AprilTag> getTypeClass() {
    return AprilTag.class;
  }

  @Override
  public String getTypeString() {
    return "struct:AprilTag";
  }

  @Override
  public int getSize() {
    return kSizeInt8 + Pose3d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "uint8 id;Pose3d pose";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose3d.struct};
  }

  @Override
  public AprilTag unpack(ByteBuffer bb) {
    int id = (int) bb.get();
    Pose3d pose = Pose3d.struct.unpack(bb);
    return new AprilTag(id, pose);
  }

@Override
  public void pack(ByteBuffer bb, AprilTag value) {
    bb.put((byte) value.ID);
    Pose3d.struct.pack(bb, value.pose);
  }
}
