package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.AllianceFlipUtil;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.vision.ApriltagVision;

public class CommandRotateToPose extends Command {

  private final CommandSwerveDrivetrain swerve;
  private Rotation2d desiredAngle;
  private boolean fixedRotation;
  private ApriltagVision m_vision;
  SwerveRequest.FieldCentric drive;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  public CommandRotateToPose(CommandSwerveDrivetrain swerve, Rotation2d r) {
    this.swerve = swerve;
    this.desiredAngle = r;
    this.fixedRotation = true;

    thetaController.setTolerance(Units.degreesToRadians(0.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  public CommandRotateToPose(CommandSwerveDrivetrain swerve, ApriltagVision m_vision) {
    this.swerve = swerve;
    this.fixedRotation = false;
    this.m_vision = m_vision;
    thetaController.setTolerance(Units.degreesToRadians(0.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Constants.Vision.autoAlignActice = true;
    if (!fixedRotation){
      switch (ScoringConstants.currentScoringMode) {
        case PODIUM:
          this.desiredAngle= Constants.SATConstants.PODIUM.pose.getRotation();
          break;
        case AMP:
          this.desiredAngle= Constants.SATConstants.AMP.pose.getRotation();
          break;
        default:
          break;
      }
    }

      
    var currPose = swerve.getState().Pose;
    this.desiredAngle = Rotation2d.fromDegrees(currPose.getRotation().getRadians() - Units.degreesToRadians(m_vision.getYaw()));
    SmartDashboard.putNumber("desiredAngle", desiredAngle.getDegrees());  
    thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currPose = swerve.getState().Pose;
    double thetaVelocity =
        thetaController.calculate(
            currPose.getRotation().getRadians(), desiredAngle.getRadians());

    if (atGoal()) {
      thetaVelocity = 0.0;
    }

    // swerve.applyRequest(
    //     () -> drive.withRotationalRate(thetaVelocity)
    // );

  }

  public boolean atGoal() {
    return thetaController.atGoal();
  }

  @Override
  public boolean isFinished(){
    return atGoal();
    // return true;
  }

  @Override
  public void end(boolean interrupted){
    Constants.Vision.autoAlignActice = false;
  }
}