// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.guitar;

public class SubGuitar extends SubsystemBase {
  public static final class Guitar {
    //define the intake
    public static final class Intake {
      public static TalonSRX Motor = new TalonSRX(Constants.guitar.intake.motor.canID);
      public static DigitalInput IoNoteInPlace; // photoeye on the intake -- normally closed
      public static boolean NoteInPlace; // photoeye on the intake -- normally closed
    }

    //define the upper shooter roller motor
    public static final class ShooterUpper {
      public static TalonFX Motor = new TalonFX(Constants.guitar.shooterUpper.motor.canID, "canivore");
    }

    //define the lower shooter roller motor
    public static final class ShooterLower {
      public static TalonFX Motor = new TalonFX(Constants.guitar.shooterLower.motor.canID, "canivore");
    }

    //define guitar pivot
    public static final class Pivot {
      public static TalonFX MotorLeft = new TalonFX(Constants.guitar.pivot.motorLeft.canID, "canivore");
      public static TalonFX MotorRight = new TalonFX(Constants.guitar.pivot.motorRight.canID, "canivore");
      public static CANcoder Encoder = new CANcoder(Constants.guitar.pivot.encoder.canID, "canivore");
      // public static DigitalInput IoHome; // guitar pivot home position limit switch -- normally closed
      // public static boolean IsHome; // guitar pivot home position limit switch -- normally closed
    }
  }

  //declare other variables
 
  /** Creates a new shooter subsystem */
  public SubGuitar() {
    final DutyCycleOut dutyCycleZero = new DutyCycleOut(0);

    //initializes the intake
    Guitar.Intake.Motor.configFactoryDefault();
    Guitar.Intake.Motor.set(TalonSRXControlMode.PercentOutput, 0);
    Guitar.Intake.Motor.setNeutralMode(NeutralMode.Brake);
    Guitar.Intake.Motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Tachometer, 0, 10);
    Guitar.Intake.Motor.setInverted(Constants.guitar.intake.motor.bInverted);
    Guitar.Intake.IoNoteInPlace = new DigitalInput(Constants.guitar.intake.ioNoteInPlace);

    //initializes the upper shooter roller motor
    var shooterUpperConfig = new TalonFXConfiguration();
    shooterUpperConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    shooterUpperConfig.Slot0.kP = Constants.guitar.shooterUpper.motor.kP;
    shooterUpperConfig.Slot0.kI = Constants.guitar.shooterUpper.motor.kI;
    shooterUpperConfig.Slot0.kD = Constants.guitar.shooterUpper.motor.kD;
    shooterUpperConfig.Slot0.kV = Constants.guitar.shooterUpper.motor.kV;
    // shooterUpperConfig.CurrentLimits.StatorCurrentLimit = 150;
    Guitar.ShooterUpper.Motor.getConfigurator().apply(shooterUpperConfig);
    Guitar.ShooterUpper.Motor.setControl(dutyCycleZero);
    Guitar.ShooterUpper.Motor.setInverted(Constants.guitar.shooterUpper.motor.bInverted);
    Guitar.ShooterUpper.Motor.setNeutralMode(NeutralModeValue.Coast);
    
    //initializes the lower shooter roller motor
    var shooterLowerConfig = new TalonFXConfiguration();
    shooterLowerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    shooterLowerConfig.Slot0.kP = Constants.guitar.shooterLower.motor.kP;
    shooterLowerConfig.Slot0.kI = Constants.guitar.shooterLower.motor.kI;
    shooterLowerConfig.Slot0.kD = Constants.guitar.shooterLower.motor.kD;
    shooterLowerConfig.Slot0.kV = Constants.guitar.shooterLower.motor.kV;
    // shooterLowerConfig.CurrentLimits.StatorCurrentLimit = 150;
    Guitar.ShooterLower.Motor.getConfigurator().apply(shooterLowerConfig);
    Guitar.ShooterLower.Motor.setControl(dutyCycleZero);
    Guitar.ShooterLower.Motor.setInverted(Constants.guitar.shooterLower.motor.bInverted);
    Guitar.ShooterLower.Motor.setNeutralMode(NeutralModeValue.Coast);
    
    //initializes the left guitar pivot motor -- THE RIGHT MOTOR FOLLOWS THE LEFT
    var guitarPivotLeftConfig = new TalonFXConfiguration();
    guitarPivotLeftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    guitarPivotLeftConfig.Feedback.FeedbackRemoteSensorID = Guitar.Pivot.Encoder.getDeviceID();
    guitarPivotLeftConfig.Slot0.kP = Constants.guitar.pivot.motorLeft.kP;
    guitarPivotLeftConfig.Slot0.kI = Constants.guitar.pivot.motorLeft.kI;
    guitarPivotLeftConfig.Slot0.kD = Constants.guitar.pivot.motorLeft.kD;
    // guitarPivotLeftConfig.CurrentLimits.StatorCurrentLimit = 150;
    guitarPivotLeftConfig.Slot0.kG = Constants.guitar.pivot.motorLeft.kG;
    guitarPivotLeftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    Guitar.Pivot.MotorLeft.getConfigurator().apply(guitarPivotLeftConfig);
    Guitar.Pivot.MotorLeft.setControl(dutyCycleZero);
    Guitar.Pivot.MotorLeft.setNeutralMode(NeutralModeValue.Brake);
    Guitar.Pivot.MotorLeft.setInverted(Constants.guitar.pivot.motorLeft.bInverted);

    //initializes the right guitar pivot motor -- THIS IS SET TO FOLLOW THE LEFT
    var guitarPivotRightConfig = new TalonFXConfiguration();
    guitarPivotRightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    guitarPivotRightConfig.Feedback.FeedbackRemoteSensorID = Guitar.Pivot.Encoder.getDeviceID();
    // guitarPivotRightConfig.CurrentLimits.StatorCurrentLimit = 150;
    Guitar.Pivot.MotorRight.getConfigurator().apply(guitarPivotRightConfig);
    Guitar.Pivot.MotorRight.setControl(dutyCycleZero);
    Guitar.Pivot.MotorRight.setNeutralMode(NeutralModeValue.Brake);
    // guitar.pivot.MotorRight.setInverted(Constants.Guitar.Pivot.MotorRight.bInverted);
    Guitar.Pivot.MotorRight.setControl(new Follower(Guitar.Pivot.MotorLeft.getDeviceID(), true));

    //initializes the standalone encoder
    var guitarEncoderConfig = new CANcoderConfiguration();
    guitarEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    guitarEncoderConfig.MagnetSensor.MagnetOffset = Constants.guitar.pivot.encoder.dMagnetOffset;
    guitarEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    Guitar.Pivot.Encoder.getConfigurator().apply(guitarEncoderConfig);
    // Guitar.Pivot.IoHome = new DigitalInput(Constants.guitar.pivot.ioHome);
  }

  /** Stops the shooter intake roller motor */
  public void stopIntake() {
    Guitar.Intake.Motor.set(ControlMode.PercentOutput, 0);
    Guitar.Intake.Motor.setNeutralMode(NeutralMode.Brake);
  }

  /** Allows the operator to run the shooter intake roller motor at the speed passed in from the command
   * @param targetSpeed (-1.0 to 1.0) % speed to run the motor -- positive values move notes towards the upper & lower shoot rollers, negative values move them away
  */
  public void runIntakeAtSpeed(double targetSpeed) {
    Guitar.Intake.Motor.set(ControlMode.PercentOutput, targetSpeed);
  }

  /** Stops the shooter motors */
  public void stopShooter(){
    var dutyCycleZero = new DutyCycleOut(0);
    Guitar.ShooterUpper.Motor.setControl(dutyCycleZero);
    Guitar.ShooterLower.Motor.setControl(dutyCycleZero);
  }

  /** Allows the operator to run the shooter upper roller motor at the speed passed in from the command
   * @param targetSpeed peed to run the motor -- positive values throw notes out the front, negative values push them back towards the intake rollers
  */
  public void runShooterUpperAtSpeed(double targetSpeed) {
    var veltarget = new VelocityDutyCycle(targetSpeed);
    Guitar.ShooterUpper.Motor.setControl(veltarget);
  }

  /** Allows the operator to run the shooter lower roller motor at the speed passed in from the command
   * @param targetSpeed speed to run the motor -- positive values throw notes out the front, negative values push them back towards the intake rollers
  */
  public void runShooterLowerAtSpeed(double targetSpeed) {
    var veltarget = new VelocityDutyCycle(targetSpeed);
    Guitar.ShooterLower.Motor.setControl(veltarget);
  }

  /** Sets the guitar pivot's standalone encoder's current position to the value of the home constant
   * you should probably never use this
  */
  public void setPivotHome(double posHome) {
    Guitar.Pivot.Encoder.setPosition(posHome);
  }

  /** Runs the guitar pivot at the speed passed in from the command -- THIS HAS NO SOFT POSITION LIMITS
   * @param targetSpeed (-1.0 to 1.0) % speed to run the motor -- negative values lower the pivot, positive values raise it
  */
  public void runPivotAtSpeed(double targetSpeed) {
    var targetDutyCycle = new DutyCycleOut(targetSpeed);
    Guitar.Pivot.MotorLeft.setControl(targetDutyCycle);
   }
  
  /** Moves the guitar pivot to the target position passed in from the command
   * @param targetPosition target position for the pivot to reach in pivot shaft rotations
  */
  public void movePivotLeftToPosition(double targetPosition) {
    var positionRequested = new PositionDutyCycle(targetPosition);
    Guitar.Pivot.MotorLeft.setControl(positionRequested); // command the motor to reach the target position
  }

  public double getShooterUpperSpeed() {
    return Guitar.ShooterUpper.Motor.getVelocity().getValue();
  }

  public double getShooterLowerSpeed() {
    return Guitar.ShooterLower.Motor.getVelocity().getValue();
  }

  public boolean shooterIsAtSpeed(double upperTargetSpeed, double lowerTargetSpeed) {
    if(MathUtil.isNear(upperTargetSpeed, getShooterUpperSpeed(), guitar.shooterUpper.motor.velTolerance) 
    && MathUtil.isNear(lowerTargetSpeed, getShooterLowerSpeed(), guitar.shooterLower.motor.velTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getIntakeHasNote() {
    return !Guitar.Intake.IoNoteInPlace.get();
  }

  public double getGuitarPivotPosition() {
    return Guitar.Pivot.Encoder.getPosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note In Place", getIntakeHasNote());
    // SmartDashboard.putBoolean("Guitar Pivot Home Switch", !Guitar.Pivot.IoHome.get());

    // SmartDashboard.putNumber("Guitar Pivot Position", getGuitarPivotPosition());
    
    // SmartDashboard.putNumber("Shoot RPM - Upper", getShooterUpperSpeed());
    // SmartDashboard.putNumber("Shoot RPM - Lower", getShooterLowerSpeed());

  } 
}