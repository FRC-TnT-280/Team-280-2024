// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.climberLeft;
import frc.robot.Constants.climberRight;
// import frc.robot.commands.Candle.LEDsToAlliance;
import frc.robot.subsystems.SubCandle;
import frc.robot.subsystems.SubClimber;
// import frc.robot.subsystems.SubSwitchPanel;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // UsbCamera camera0 = CameraServer.startAutomaticCapture(0);
    // camera0.setResolution(320, 240);
    // camera0.setFPS(10);

    SubClimber.ClimberLeft.Hook.Motor.setPosition(climberLeft.hook.motor.posDownFull);
    SubClimber.ClimberRight.Hook.Motor.setPosition(climberRight.hook.motor.posDownFull);

    SubClimber.ClimberLeft.Pivot.Encoder.setPosition(0);
    SubClimber.ClimberRight.Pivot.Encoder.setPosition(0);

    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Blue) {
      SubCandle.m_candle.setLEDs(0,0,255);
    } else if (alliance.get() == DriverStation.Alliance.Red) {
      SubCandle.m_candle.setLEDs(255, 0, 0);
    }

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
