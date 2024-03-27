// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// swerve stuff
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.targetlock;
import frc.robot.commands.Climber.CHooksGoUp;
import frc.robot.commands.Climber.CHooksGoDown;
import frc.robot.commands.Climber.CPivotsGoTo;
import frc.robot.commands.Guitar.GPivotGoTo;
import frc.robot.commands.Guitar.IntakeNote;
import frc.robot.commands.Guitar.ReverseNote;
import frc.robot.commands.Guitar.ShootNoteAt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SubCandle;
import frc.robot.subsystems.SubClimber;
import frc.robot.subsystems.SubGuitar;
import frc.robot.subsystems.SubSwitchPanel;

// misc
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.climberLeft;
import frc.robot.Constants.climberRight;
import frc.robot.Constants.guitar;
import frc.robot.Constants.guitar.shooterLower;
import frc.robot.Constants.guitar.shooterUpper;

public class RobotContainer {
  //Subsystems
  private final SubGuitar s_Guitar = new SubGuitar();
  private final SubClimber s_Climber = new SubClimber();
  private final Limelight s_Limelight = new Limelight();
  private final SubCandle s_Candle = new SubCandle();
  private final SubSwitchPanel s_SwitchPanel = new SubSwitchPanel();
  
  private final CommandXboxController driverXboxController = new CommandXboxController(0); // driver's xbox controller
  private final CommandXboxController operatorXboxController = new CommandXboxController(2); // operator's xbox controller
  
  /* SWERVE STUFF */
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    // Field centric driving in closed loop with 10% deadband
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.1)
        .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(TunerConstants.MaxSpeed * 0.1)
        .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Field centric driving in closed loop with target locking and 10% deadband
    private final targetlock targetLock = new targetlock(s_Limelight, drivetrain::getHeading)
        .withDeadband(TunerConstants.MaxSpeed * 0.1)
        .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);
  /* SWERVE STUFF END */

  //Misc declarations

  //Auton Chooser
  private final SendableChooser<Command> autoChooser;

  // the robit
  public RobotContainer() {
    // Pathplanner command registration
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(s_Guitar, s_Candle, s_SwitchPanel));
    NamedCommands.registerCommand("ShootNoteAtDefault", new ShootNoteAt(s_Guitar, s_Candle, s_SwitchPanel, shooterUpper.motor.velDefaultRange, shooterLower.motor.velDefaultRange));
    NamedCommands.registerCommand("ShootNoteAtDenial", new ShootNoteAt(s_Guitar, s_Candle, s_SwitchPanel, shooterUpper.motor.velClearNote, shooterLower.motor.velClearNote));
    // NamedCommands.registerCommand("GuitarToHome", new InstantCommand(() -> s_Guitar.movePivotLeftToPosition(guitar.pivot.posHome)));
    NamedCommands.registerCommand("GuitarToHome", new GPivotGoTo(s_Guitar, guitar.pivot.posHome));
    NamedCommands.registerCommand("AllianceColors", new InstantCommand(() -> s_Candle.setLED_ToAlliance(s_SwitchPanel)));
    
    // SmartDashboard.putData(s_Climber);
    // SmartDashboard.putData(s_Guitar);

    configureBindings();
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private void configureBindings() { 
    /** DRIVER CONTROLS
     * LEFT STICK --  X/Y swerve
     * RIGHT STICK  --  robot rotation
     * A  --  reset field orientation
     * B  --  
     * X
     * Y
     * LT -- (hold) swerve drive mode = limglight target lock
     * LB --  (hold) swerve drive mode = robot-centric
     * RT --  (hold) intake note
     * RB -- (hold) reverse note out
     * START
     * BACK
     * POV UP (0)
     * POV RIGHT (90)
     * POV DOWN (180)
     * POV LEFT (270)
     * LEFT STICK CLICK
     * RIGHT STICK CLICK
     */
 
    // select drive mode
    drivetrain.setDefaultCommand(drivetrain.applyRequestWithName(
      () -> drive
          .withVelocityX(-driverXboxController.getLeftY() * TunerConstants.MaxSpeed)
          .withVelocityY(-driverXboxController.getLeftX() * TunerConstants.MaxSpeed)
          .withRotationalRate(-driverXboxController.getRightX() * TunerConstants.MaxAngularRate),
      "Default drive"));

    // driverXboxController.leftTrigger().whileTrue(drivetrain.applyRequestWithName(
    //   () -> targetLock
    //       .withVelocityX(-driverXboxController.getLeftY() * TunerConstants.MaxSpeed)
    //       .withVelocityY(-driverXboxController.getLeftX() * TunerConstants.MaxSpeed),
    //   "Target lock"));
    
    driverXboxController.leftTrigger().whileTrue(drivetrain.applyRequestWithName(
      () -> robotCentricDrive
          .withVelocityX(driverXboxController.getLeftY() * TunerConstants.MaxSpeed * 0.5)
          .withVelocityY(driverXboxController.getLeftX() * TunerConstants.MaxSpeed * 0.5)
          .withRotationalRate(-driverXboxController.getRightX() * TunerConstants.MaxAngularRate * 0.75),
      "Robot centric drive"));      

    // LED TEST COMMANDS
    // driverXboxController.povUp()
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_L3(000, 000, 255)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_L2(255, 000, 000)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_L1(255, 255, 255)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_C0(000, 000, 255)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_R1(255, 255, 255)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_R2(255, 000, 000)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_R3(000, 000, 255)))
    //   .onTrue(new InstantCommand(() -> s_Candle.setLED_Candle(255, 255, 255)));

    // driverXboxController.povDown().onTrue(new InstantCommand(() -> s_Candle.setLED_All(255, 0, 0)));
    // driverXboxController.povRight().onTrue(new InstantCommand(() -> s_Candle.setLED_All(225, 10, 0)));
    // driverXboxController.povLeft().onTrue(new InstantCommand(() -> s_Candle.setLED_All(30, 0, 255)));

    // reset the field-centric heading on A button
    driverXboxController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // intake note
    driverXboxController.rightTrigger()
      .onTrue(new IntakeNote(s_Guitar, s_Candle, s_SwitchPanel));
      // .whileTrue(new IntakeNote(s_Guitar, s_Candle, s_SwitchPanel));

    driverXboxController.rightBumper()
      .onTrue (new InstantCommand(() -> s_Guitar.stopIntake()))
      .whileTrue(new ReverseNote(s_Guitar, s_Candle, s_SwitchPanel));

    /** OPERATOR CONTROLS
     * LEFT STICK -- 
     * RIGHT STICK  --  
     * A  --  guitar to floor
     * B  --  guitar to amp
     * X  -- guitar to mid range shot
     * Y  -- guitar to trap
     * LT -- (hold) guitar to limelight position
     * LB --  (hold) shoot note just in front of the bot
     * RT --  (hold) shoot with limelight range
     * RB --  (hold) shoot with default range
     * START  --  climbers up
     * BACK --  climbers down
     * POV UP (0) --  (hold) hooks up
     * POV RIGHT (90) --
     * POV DOWN (180) -- (hold) hooks down
     * POV LEFT (270) --  
     * LEFT STICK CLICK --  
     * RIGHT STICK CLICK  --  
     */

    // hooks up (hold) 
    operatorXboxController.povUp()
      .whileTrue(new CHooksGoUp(s_Climber));
      // .onFalse(new InstantCommand(() -> s_Climber.runClimberLeftHookAtSpeed(0)))
      // .onFalse(new InstantCommand(() -> s_Climber.runClimberRightHookAtSpeed(0)));

    // hooks down (hold)
    operatorXboxController.povDown()
      .whileTrue(new CHooksGoDown(s_Climber));
      // .onFalse(new InstantCommand(() -> s_Climber.runClimberLeftHookAtSpeed(0)))
      // .onFalse(new InstantCommand(() -> s_Climber.runClimberRightHookAtSpeed(0)));

    // climber pivots to climb 
    operatorXboxController.start()
      .onTrue(new CPivotsGoTo(s_Climber, climberLeft.pivot.posClimb, climberRight.pivot.posClimb))
      .onTrue(new CHooksGoUp(s_Climber));
      
    // climber pivots to home
    // operatorXboxController.back()
    //   .onTrue(new CPivotsGoTo(s_Climber, climberLeft.pivot.posHome, climberRight.pivot.posHome));

    //guitar pivot to trap
    operatorXboxController.y()
      .onTrue(new GPivotGoTo(s_Guitar, guitar.pivot.posTrap));

    // guitar pivot to floor
    operatorXboxController.a()
      .onTrue(new GPivotGoTo(s_Guitar, guitar.pivot.posHome));

    // // guitar pivot to mid range shot
    // operatorXboxController.x()
    //   .onTrue(new GPivotGoTo(s_Guitar, guitar.pivot.posMid));

    // guitar pivot to amp
    operatorXboxController.b()
      .onTrue(new GPivotGoTo(s_Guitar, guitar.pivot.posAmp));

    // // guitar pivot to limelight position
    // operatorXboxController.leftTrigger()
    //   .onTrue(new GPivotGoTo(s_Guitar, WHATEVER THE LIMELIGHT SAYS);)

    // // shoot note at limelight range
    // operatorXboxController.rightTrigger()
    //   .onTrue(new ShootNoteAt(s_Guitar, LIMELIGHT UPPER SPEED, LIMELIGHT LOWER SPEED));

    // shoot note at default range
    operatorXboxController.rightBumper()
      .onTrue(new ShootNoteAt(s_Guitar, s_Candle, s_SwitchPanel, shooterUpper.motor.velDefaultRange, shooterLower.motor.velDefaultRange));

    // clear note over the top of the bot
    operatorXboxController.leftBumper()
      .onTrue(new ShootNoteAt(s_Guitar, s_Candle, s_SwitchPanel, shooterUpper.motor.velClearNote, shooterLower.motor.velClearNote));
  
    // telemeterizes data if in simulation
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}