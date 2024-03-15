package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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

              if (mFrontRightAprilTagResult.targets.size() >= 2 || (mFrontRightAprilTagResult.targets.size() == 1 && mFrontRightAprilTagResult.getBestTarget().getPoseAmbiguity() < Constants.Vision.APRILTAG_AMBIGUITY_THRESHOLD)){
                mFrontRight = getBackLeftEstimatedGlobalPose(Robot.m_robotContainer.drivetrain.getState().Pose, mFrontRightAprilTagResult);

                mFrontRight.ifPresent(frontRight -> {
                  Robot.m_robotContainer.drivetrain.addVisionMeasurement(
                          new Pose2d(frontRight.estimatedPose.toPose2d().getTranslation(),
                                  Robot.m_robotContainer.drivetrain.getState().Pose.getRotation()),
                          Timer.getFPGATimestamp() - mFrontRightAprilTagResult.getLatencyMillis());
                });
              }
          }

          if (this.mBackLeftCam != null){
              mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();

              if (mBackLeftAprilTagResult.targets.size() >= 2 || (mBackLeftAprilTagResult.targets.size() == 1 && mBackLeftAprilTagResult.getBestTarget().getPoseAmbiguity() < Constants.Vision.APRILTAG_AMBIGUITY_THRESHOLD)){
                mBackLeft = getFrontRightEstimatedGlobalPose(Robot.m_robotContainer.drivetrain.getState().Pose, mBackLeftAprilTagResult);

                mBackLeft.ifPresent(backLeft -> {
                    Robot.m_robotContainer.drivetrain.addVisionMeasurement(
                            new Pose2d(backLeft.estimatedPose.toPose2d().getTranslation(),
                                    Robot.m_robotContainer.drivetrain.getState().Pose.getRotation()),
                            Timer.getFPGATimestamp() - mFrontRightAprilTagResult.getLatencyMillis());
                });
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

