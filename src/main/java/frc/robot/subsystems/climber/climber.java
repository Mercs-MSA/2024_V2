// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;


public class climber extends SubsystemBase {
  private TalonFX climberMotorLeft = new TalonFX(Constants.climberConstants.climberMotor_Left_ID);
  private TalonFX climberMotorRight = new TalonFX(Constants.climberConstants.climberMotor_Right_ID);

  private final PositionVoltage climberMotorRight_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage climberMotorLeft_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    
  private double rightMotorPosition;
  private double leftMotorPosition;

  public climberStates my_climber_state = climberStates.START;
  // private double targetPose = leftMotorPosition;
        
  /* TODO:
   * - setDirection() changes the variable depending on direction
   * - figure joysticks
   * - don't need Transit state?!
   */

  public climber() {
    TalonFXConfiguration climberRightMotorConfigs = new TalonFXConfiguration();
    climberRightMotorConfigs.Slot0.kP = 5.0; // An error of 0.5 rotations results in 1.2 volts output
    climberRightMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    climberRightMotorConfigs.Voltage.PeakForwardVoltage = 2;
    climberRightMotorConfigs.Voltage.PeakReverseVoltage = -2;

    TalonFXConfiguration climberLeftMotorConfigs = new TalonFXConfiguration();
    climberLeftMotorConfigs.Slot0.kP = 5.0; // An error of 0.5 rotations results in 1.2 volts output
    climberLeftMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    climberLeftMotorConfigs.Voltage.PeakForwardVoltage = 2;
    climberLeftMotorConfigs.Voltage.PeakReverseVoltage = -2;

    climberMotorLeft.getConfigurator().apply(climberRightMotorConfigs);
    climberMotorRight.getConfigurator().apply(climberLeftMotorConfigs);

    optimization_for_CAN();
  }
  
  public double outputRightData() {
    return rightMotorPosition;
  }

  public double outputLeftData() {
    return leftMotorPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightMotorPosition = climberMotorRight.getPosition().getValueAsDouble();
    leftMotorPosition = climberMotorLeft.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("climber right motor position", rightMotorPosition);

    SmartDashboard.putNumber("climber left motor position", leftMotorPosition);

    SmartDashboard.putNumber("Motor Right State", outputRightData());
    SmartDashboard.putNumber("Motor Left State", outputLeftData());
  }
  

  /**
   * This method prepares the robot for climbing in the middle of the chain
   */
public void resetMotors(){
    climberMotorRight.setControl(new NeutralOut());
    climberMotorLeft.setControl(new NeutralOut());
  }

public void climbStop() {
    climberMotorRight.setControl(new NeutralOut());
    climberMotorLeft.setControl(new NeutralOut());
  }

// COMP CONTROLS
// public void climbDown(){
//     climberMotorRight.setControl(climberMotorRight_voltagePosition.withPosition(Constants.climberConstants.RIGHT_BOTTOM_POSITION));
//     climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(Constants.climberConstants.LEFT_BOTTOM_POSITION));
// }

public void climb(double climberRightPos, double climberLeftPos){
    climberMotorRight.setControl(climberMotorRight_voltagePosition.withPosition(climberRightPos));
    climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(climberLeftPos));
}



// public void climbUp() {
//     climberMotorRight.setControl(climberMotorRight_voltagePosition.withPosition(Constants.climberConstants.RIGHT_TOP_POSITION));
//     climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(Constants.climberConstants.LEFT_TOP_POSITION));
// }
//MANUAL CONTROLS
public void climbRightUp() {
    climberMotorRight.setControl(climberMotorRight_voltagePosition.withPosition(rightMotorPosition + Constants.climberConstants.climber_Increment)); 
}

public void climbRightDown() {
    climberMotorRight.setControl(climberMotorRight_voltagePosition.withPosition(rightMotorPosition - Constants.climberConstants.climber_Increment)); 
}

public void climbLeftUp() { 
    climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(leftMotorPosition - Constants.climberConstants.climber_Increment));
}

public void climbLeftDown() { 
    climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(leftMotorPosition + Constants.climberConstants.climber_Increment));
}

public void incrementalClimbBothSides(double y){
    climberMotorRight.setControl(climberMotorRight_voltagePosition.withPosition(rightMotorPosition + (y*Constants.climberConstants.climber_Increment)));
    climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(leftMotorPosition + ((y*Constants.climberConstants.climber_Increment))));
  }

    // climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(leftMotorPosition + ((y*Constants.climberConstants.climber_Increment))));
  

  // public void leftGoToPosition(double joystick){
  //   targetPose = leftMotorPosition + (0.05*joystick);
  //   if (targetPose > 0 && targetPose < 14){
  //     climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(targetPose));
  //   }
  // }

  // public void leftGoToPosition(double joystick){
  //   targetPose = leftMotorPosition + (0.05*joystick);
  //   if (targetPose > 0 && targetPose < 14){
  //     climberMotorLeft.setControl(climberMotorLeft_voltagePosition.withPosition(targetPose));
  //   }
  // }

  // // Block of all commands
  // public Command climbUpCommand() {
  //   return new RunCommand(() -> climbUp(), this);
  // }

  // public Command climbDownCommand() {
  //   return new RunCommand(() -> climbDown(), this);
  // }

  public Command climbUpRightCommand() {
    return new RunCommand(() -> climbRightUp(), this);
  }

  public Command climbDownRightCommand() {
    return new RunCommand(() -> climbRightDown(), this);
  }

  public Command climbUpLeftCommand() {
    return new RunCommand(() -> climbLeftUp(), this);
  }

  public Command climbDownLeftCommand() {
    return new RunCommand(() -> climbLeftDown(), this);
  }

  public Command climbMotorStop() {
    return new RunCommand(() -> climbStop(), this);
  }
 
  // State Enumeration
  public enum climberStates {
    START, MOVING_TO_CLIMB, IN_CLIMBING_POSITION, COMPLETED_CLIMB, ERROR;
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_TubeMotorL_canbus1signal1 = climberMotorLeft.getPosition();
    StatusSignal<Double> m_TubeMotorR_canbus1signal2 = climberMotorRight.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_TubeMotorL_canbus1signal1, m_TubeMotorR_canbus1signal2);
    ParentDevice.optimizeBusUtilizationForAll(climberMotorLeft, climberMotorRight);
  }
}