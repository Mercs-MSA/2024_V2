package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AllianceFlipUtil;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Swerve;

public class CommandRotateToPose extends Command {

  private final Swerve swerve;
  private Rotation2d desiredAngle;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  public CommandRotateToPose(Swerve swerve, Rotation2d r) {
    this.swerve = swerve;
    this.desiredAngle = r;

    thetaController.setTolerance(Units.degreesToRadians(1.7));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  public CommandRotateToPose(Swerve swerve) {
    this.swerve = swerve;

    thetaController.setTolerance(Units.degreesToRadians(1.7));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Constants.Vision.autoAlignActice = true;
    switch (ScoringConstants.currentScoringMode) {
      case PODIUM:
        this.desiredAngle= Constants.SATConstants.PODIUM.pose.getRotation();
        break;
      case SUBWOOFER:
        this.desiredAngle= Constants.SATConstants.SUB.pose.getRotation();
        break;
      case AMP:
        this.desiredAngle= Constants.SATConstants.AMP.pose.getRotation();
        break;
      default:
        break;
    }

    this.desiredAngle = AllianceFlipUtil.apply(desiredAngle);    
    var currPose = swerve.getPose();
    thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currPose = swerve.getPose();
    double thetaVelocity =
        thetaController.calculate(
            currPose.getRotation().getRadians(), desiredAngle.getRadians());

    if (atGoal()) {
      thetaVelocity = 0.0;
    }

    swerve.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, thetaVelocity, currPose.getRotation()));
  }

  public boolean atGoal() {
    return thetaController.atGoal();
  }

  @Override
  public boolean isFinished(){
    return atGoal();
  }

  @Override
  public void end(boolean interrupted){
    Constants.Vision.autoAlignActice = false;
  }
}