// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.climberLeft;
import frc.robot.Constants.climberRight;

public class SubClimber extends SubsystemBase {
  
  //define the left climber
  public static final class ClimberLeft {

    //define the left climber pivot
    public static final class Pivot {
      public static TalonFX Motor = new TalonFX(Constants.climberLeft.pivot.motor.canID, "canivore");
      public static CANcoder Encoder = new CANcoder(Constants.climberLeft.pivot.encoder.canID, "canivore"); // shaft-mounted standalone encoder
    }

    //define the left climber hook
    public static final class Hook {
      public static TalonFX Motor = new TalonFX(Constants.climberLeft.hook.motor.canID, "canivore");
    }
  }

  //define the right climber
  public static final class ClimberRight {
    //define the right climber pivot
    public static final class Pivot {
      public static TalonFX Motor = new TalonFX(Constants.climberRight.pivot.motor.canID, "canivore");
      public static CANcoder Encoder = new CANcoder(Constants.climberRight.pivot.encoder.canID, "canivore"); // shaft-mounted standalone encoder
    }

    //define the right climber hook
    public static final class Hook {
      public static TalonFX Motor = new TalonFX(Constants.climberRight.hook.motor.canID, "canivore");
    }
  }
  
  /** Creates a new climber subsystem */
  public SubClimber() {
    final DutyCycleOut dutyCycleZero = new DutyCycleOut(0);

    // PIVOT INITS
    //initializes the left climber pivot motor
    var climberLeftPivotMotorConfig = new TalonFXConfiguration();
    climberLeftPivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    climberLeftPivotMotorConfig.Feedback.FeedbackRemoteSensorID = 31;
    climberLeftPivotMotorConfig.Slot0.kP = Constants.climberLeft.pivot.motor.kP;
    climberLeftPivotMotorConfig.Slot0.kI = Constants.climberLeft.pivot.motor.kI;
    climberLeftPivotMotorConfig.Slot0.kD = Constants.climberLeft.pivot.motor.kD;
    climberLeftPivotMotorConfig.CurrentLimits.StatorCurrentLimit = 150;
    ClimberLeft.Pivot.Motor.getConfigurator().apply(climberLeftPivotMotorConfig);
    ClimberLeft.Pivot.Motor.setControl(dutyCycleZero);
    ClimberLeft.Pivot.Motor.setInverted(Constants.climberLeft.pivot.motor.bInverted);
    ClimberLeft.Pivot.Motor.setNeutralMode(NeutralModeValue.Brake);

    //initializes the right climber pivot motor
    var climberRightPivotMotorConfig = new TalonFXConfiguration();
    climberRightPivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    climberRightPivotMotorConfig.Feedback.FeedbackRemoteSensorID = 34;
    climberRightPivotMotorConfig.Slot0.kP = Constants.climberRight.pivot.motor.kP;
    climberRightPivotMotorConfig.Slot0.kI = Constants.climberRight.pivot.motor.kI;
    climberRightPivotMotorConfig.Slot0.kD = Constants.climberRight.pivot.motor.kD;
    climberRightPivotMotorConfig.CurrentLimits.StatorCurrentLimit = 150;
    ClimberRight.Pivot.Motor.getConfigurator().apply(climberRightPivotMotorConfig);
    ClimberRight.Pivot.Motor.setControl(dutyCycleZero);
    ClimberRight.Pivot.Motor.setInverted(Constants.climberRight.pivot.motor.bInverted);
    ClimberRight.Pivot.Motor.setNeutralMode(NeutralModeValue.Brake);

    //initializes the left climber pivot encoder
    var climberLeftPivotEncoderConfig = new CANcoderConfiguration();
    climberLeftPivotEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO set encoder inverts if using this
    climberLeftPivotEncoderConfig.MagnetSensor.MagnetOffset = Constants.climberLeft.pivot.encoder.dMagnetOffset;
    climberLeftPivotEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    ClimberLeft.Pivot.Encoder.getConfigurator().apply(climberLeftPivotEncoderConfig);

    //initializes the right climber pivot encoder
    var climberRightPivotEncoderConfig = new CANcoderConfiguration();
    ClimberRight.Pivot.Encoder.getConfigurator().apply(climberRightPivotEncoderConfig);
    climberRightPivotEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // TODO set encoder inverts if using this
    climberRightPivotEncoderConfig.MagnetSensor.MagnetOffset = Constants.climberRight.pivot.encoder.dMagnetOffset;
    climberRightPivotEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    ClimberRight.Pivot.Encoder.getConfigurator().apply(climberRightPivotEncoderConfig);

    // HOOK INITS
    //initializes the left climber hook motor
    var climberLeftHookConfig = new TalonFXConfiguration();
    climberLeftHookConfig.CurrentLimits.StatorCurrentLimit = 150;
    climberLeftHookConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    ClimberLeft.Hook.Motor.getConfigurator().apply(climberLeftHookConfig);
    ClimberLeft.Hook.Motor.setInverted(Constants.climberLeft.pivot.motor.bInverted);
    ClimberLeft.Hook.Motor.setNeutralMode(NeutralModeValue.Brake);
    ClimberLeft.Hook.Motor.setControl(dutyCycleZero);

    //initializes the right climber hook motor
    var climberRightHookConfig = new TalonFXConfiguration();
    climberRightHookConfig.CurrentLimits.StatorCurrentLimit = 150;
    climberRightHookConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    ClimberRight.Hook.Motor.getConfigurator().apply(climberRightHookConfig);
    ClimberRight.Hook.Motor.setInverted(Constants.climberRight.pivot.motor.bInverted);
    ClimberRight.Hook.Motor.setNeutralMode(NeutralModeValue.Brake);;
    ClimberRight.Hook.Motor.setControl(dutyCycleZero);
  }

  /** Sets the left climber hook motor's current position to the value of Constants.climberLeft.Hook.Motor.posHome
   * This should only be used when re-initializing the robot
   */
  public void setClimberLeftHookHome(double posHome) {
    ClimberLeft.Hook.Motor.setPosition(posHome);
  }

  /** Sets the right climber hook motor's current position to the value of Constants.climberRight.Hook.Motor.posHome
   * This should only be used when re-initializing the robot
   */
  public void setClimberRightHookHome(double posHome) {
    ClimberRight.Hook.Motor.setPosition(posHome);
  }

  /** Moves the left climber pivot to the target position passed in from the command
   * @param targetPosition target position for the pivot to reach in degrees
  */
  public void moveClimberLeftToPosition(double targetPosition) {
    var positionRequested = new PositionDutyCycle(targetPosition);
    ClimberLeft.Pivot.Motor.setControl(positionRequested);
  }

  /** Moves the right climber pivot to the target position passed in from the command
   * @param targetPosition target position for the pivot to reach in degrees
  */
  public void moveClimberRightToPosition(double targetPosition) {
    var positionRequested = new PositionDutyCycle(targetPosition);
    ClimberRight.Pivot.Motor.setControl(positionRequested);
  }

  public void runClimberLeftAtSpeed(double targetSpeed) {
    var targetDutyCycle = new DutyCycleOut(targetSpeed);
    ClimberLeft.Pivot.Motor.setControl(targetDutyCycle);
  }

  public void runClimberRightAtSpeed(double targetSpeed) {
    var targetDutyCycle = new DutyCycleOut(targetSpeed);
    ClimberRight.Pivot.Motor.setControl(targetDutyCycle);
  }

  /** Allows the operator to run the left climber hook motor at the speed passed in from the command -- THIS HAS NO SOFT POSITION LIMITS
   * @param targetSpeed (-1.0 to 1.0) % speed to run the motor -- negative values lower, positive values raise
  */
  public void runClimberLeftHookAtSpeed(double targetSpeed) {
    var targetDutyCycle = new DutyCycleOut(targetSpeed);
    ClimberLeft.Hook.Motor.setControl(targetDutyCycle);
  }

  /** Allows the operator to run the right climber hook motor at the speed passed in from the command -- THIS HAS NO SOFT POSITION LIMITS
   * @param targetSpeed (-1.0 to 1.0) % speed to run the motor -- negative values lower, positive values raise
  */
  public void runClimberRightHookAtSpeed(double targetSpeed) {
    var targetDutyCycle = new DutyCycleOut(targetSpeed);
    ClimberRight.Hook.Motor.setControl(targetDutyCycle);
  }

  public double getClimberLeftPivotPosition() {
    return ClimberLeft.Pivot.Encoder.getPosition().getValue();
  }

  public double getClimberRightPivotPosition() {
    return ClimberRight.Pivot.Encoder.getPosition().getValue();
  }

  public double getClimberLeftHookPosition() {
    return ClimberLeft.Hook.Motor.getPosition().getValue();
  }

  public double getClimberRightHookPosition() {
    return ClimberRight.Hook.Motor.getPosition().getValue();
  }

  public boolean getClimberLeftHookIsAtMaxHeight() {
    if (MathUtil.isNear(climberLeft.hook.motor.posUpFull, getClimberLeftHookPosition(), climberLeft.hook.motor.posTolerance)) {
      return true;
    } else {
      return false;
    }
  }
  
  public boolean getClimberRightHookIsAtMaxHeight() {
    if (MathUtil.isNear(climberRight.hook.motor.posUpFull, getClimberRightHookPosition(), climberRight.hook.motor.posTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getClimberLeftHookIsAtMinHeight() {
    if (MathUtil.isNear(climberLeft.hook.motor.posDownFull, getClimberLeftHookPosition(), climberLeft.hook.motor.posTolerance)) {
      return true;
    } else {
      return false;
    }
  }
  
  public boolean getClimberRightHookIsAtMinHeight() {
    if (MathUtil.isNear(climberRight.hook.motor.posDownFull, getClimberRightHookPosition(), climberRight.hook.motor.posTolerance)) {
      return true;
    } else {
      return false;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left Climber Hook At Max", getClimberLeftHookIsAtMaxHeight());
    SmartDashboard.putBoolean("Right Climber Hook At Max", getClimberRightHookIsAtMaxHeight());
  
    SmartDashboard.putBoolean("Left Climber Hook At Min", getClimberLeftHookIsAtMinHeight());
    SmartDashboard.putBoolean("Right Climber Hook At Min", getClimberRightHookIsAtMinHeight());

    SmartDashboard.putNumber("Left Climber Pivot", getClimberLeftPivotPosition());
    SmartDashboard.putNumber("Right Climber Pivot", getClimberRightPivotPosition());
    
    SmartDashboard.putNumber("Left Climber Hook", getClimberLeftHookPosition());
    SmartDashboard.putNumber("Right Climber Hook", getClimberRightHookPosition());

  } 
}