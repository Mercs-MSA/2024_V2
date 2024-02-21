// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class Intake extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = Robot.isSimulation();

  private boolean isUpperNotePresent;
  private boolean isLowerNotePresent1;
  private boolean isLowerNotePresent2;
  private boolean isLowerNotePresent3;
  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId); //carpet
  private final TalonFX indexMotor = new TalonFX(IntakeConstants.kIndexMotorId); //sat (feeder)
  private final DutyCycleOut intakeMotor_dutyCycleOut = new DutyCycleOut(0);
  private final DutyCycleOut indexMotor_dutyCycleOut = new DutyCycleOut(0);
  private final PositionVoltage intakeMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage indexMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final DigitalInput intakeUpperSensor = new DigitalInput(IntakeConstants.kIntakeUpperSensorId);
  private final DigitalInput intakeSensor1 = new DigitalInput(IntakeConstants.kIntakeLowerSensor1Id);
  private final DigitalInput intakeSensor2 = new DigitalInput(IntakeConstants.kIntakeLowerSensor2Id);
  private final DigitalInput intakeSensor3 = new DigitalInput(IntakeConstants.kIntakeLowerSensor3Id);

  private double intakeMotorPos;
  private double indexMotorPos;
  private double intakeMotorSpeed;
  private double indexMotorSpeed;

  /** Creates a new intake. */
  public Intake() {
    SmartDashboard.putString("sensor debug", "init");

    isUpperNotePresent = false;
    isLowerNotePresent1 = false;
    isLowerNotePresent2 = false;
    isLowerNotePresent3 = false;

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(configs);
      status = indexMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);
  }

  public boolean lowerSensorDetectsNote() {
    return isLowerNotePresent1 || isLowerNotePresent2 || isLowerNotePresent3;
  }

  public boolean upperSensorDetectsNote() {
    return isUpperNotePresent;
  }

  public void setLowerSensorDetectsNote(boolean value) {
    isLowerNotePresent1 = value;
    isLowerNotePresent2 = value;
    isLowerNotePresent3 = value;
  }
  
  public void setUpperSensorDetectsNote(boolean value) {
    isUpperNotePresent = value;
  }

  public void intakeMotorToPosition(double rotations) {
    intakeMotor.setControl(intakeMotor_voltagePosition.withPosition(rotations));
  }

  public void indexMotorToPosition(double rotations) {
    indexMotor.setControl(indexMotor_voltagePosition.withPosition(rotations));
  }

  public double getIntakeMotorPosition() {
    return intakeMotorPos;
  }

  public double getIndexMotorPosition() {
    return indexMotorPos;
  }

  public void startIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStart
    intakeMotor.setControl(intakeMotor_dutyCycleOut.withOutput(-IntakeConstants.kIntakeMotorSpeed));
  }

  // public void reverseIntakeMotor() {
  //   intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  // }

  public void startIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStart
    indexMotor.setControl(indexMotor_dutyCycleOut.withOutput(-IntakeConstants.kIndexMotorSpeed));
  }

  public void startIntakeIndexerMotors(){
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStart & CommandOverrideIndexStart
    startIndexMotor();
    startIntakeMotor();
  }

  public void stopIntakeIndexerMotors(){
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStop & CommandOverrideIndexStop
    stopIntakeMotor();
    stopIndexMotor();
  }

  public void stopIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStop
    intakeMotor.setControl(new NeutralOut());
  }

  public void stopIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStop
    indexMotor.setControl(new NeutralOut());
  }

  public double getIntakeMotorSpeed() {
    return intakeMotorSpeed;
  }

  public double getIndexMotorSpeed() {
    return indexMotorSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 

    if (!simulationDebugMode) {
      isUpperNotePresent = !intakeUpperSensor.get();
      isLowerNotePresent1 = !intakeSensor1.get();
      isLowerNotePresent2 = !intakeSensor2.get();
      isLowerNotePresent3 = !intakeSensor3.get();
    }
  
    intakeMotorPos = intakeMotor.getPosition().getValueAsDouble();
    indexMotorPos = indexMotor.getPosition().getValueAsDouble();
    intakeMotorSpeed = intakeMotor.getDutyCycle().getValueAsDouble();
    indexMotorSpeed = indexMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putBoolean("Upper Sensor state", isUpperNotePresent);
    SmartDashboard.putBoolean("Lower Sensor1 state", isLowerNotePresent1);
    SmartDashboard.putBoolean("Lower Sensor2 state", isLowerNotePresent2);
    SmartDashboard.putBoolean("Lower Sensor3 state", isLowerNotePresent3);
    SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Index Motor Temperature", indexMotor.getDeviceTemp().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_IntakeMotor_canbus1signal1 = intakeMotor.getPosition();
    StatusSignal<Double> m_IndexMotor_canbus1signal2 = indexMotor.getPosition();
    StatusSignal<Double> m_IntakeTemp_canbus1signal1 = intakeMotor.getDeviceTemp();
    StatusSignal<Double> m_IndexTemp_canbus1signal2 = indexMotor.getDeviceTemp();
    StatusSignal<Double> m_IntakeDutyCycle_canbus1signal1 = intakeMotor.getDutyCycle();
    StatusSignal<Double> m_IndexDutyCycle_canbus1signal2 = indexMotor.getDutyCycle();
    StatusSignal<Double> m_Shooter1Volt_canbus1signal3 = intakeMotor.getMotorVoltage();
    StatusSignal<Double> m_Shooter2Volt_canbus1signal4 = indexMotor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_IntakeMotor_canbus1signal1, m_IndexMotor_canbus1signal2, m_IntakeDutyCycle_canbus1signal1, m_IndexDutyCycle_canbus1signal2, m_Shooter1Volt_canbus1signal3, m_Shooter2Volt_canbus1signal4);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IntakeTemp_canbus1signal1, m_IndexTemp_canbus1signal2);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, indexMotor);
  }
}
