package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class CommandRotateToPose extends Command {

  private final Swerve swerve;
  private Pose2d desiredPose;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  public CommandRotateToPose(Swerve swerve, Pose2d pose) {
    this.swerve = swerve;
    this.desiredPose = pose;

    thetaController.setTolerance(Units.degreesToRadians(1));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    var currPose = swerve.getPose();
    thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currPose = swerve.getPose();
    var targetPose = desiredPose;
    double thetaVelocity =
        thetaController.calculate(
            currPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

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
}