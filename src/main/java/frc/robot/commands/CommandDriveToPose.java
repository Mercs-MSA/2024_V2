package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class CommandDriveToPose extends Command {

  private final CommandSwerveDrivetrain swerve;
  private Pose2d desiredPose;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3.2, 0, 0, new TrapezoidProfile.Constraints(4, 3.2));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(3.2, 0, 0, new TrapezoidProfile.Constraints(4, 3.2));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  public CommandDriveToPose(CommandSwerveDrivetrain swerve, Pose2d pose) {
    this.swerve = swerve;
    this.desiredPose = pose;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(2.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    var currPose = swerve.getState().Pose;
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
    thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currPose = swerve.getState().Pose;
    var targetPose = desiredPose;

    double xvelocity = xController.calculate(currPose.getX(), targetPose.getX());
    double yvelocity = yController.calculate(currPose.getY(), targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (atGoal()) {
      xvelocity = 0.0;
      yvelocity = 0.0;
      thetaVelocity = 0.0;
    }

    swerve.setControl(swerve.AutoRequest.withSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xvelocity, yvelocity, thetaVelocity, currPose.getRotation())));
  }

  public boolean atGoal() {
    return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  @Override
  public boolean isFinished(){
    return atGoal();
  }
}