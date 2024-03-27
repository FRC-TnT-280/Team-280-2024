// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Guitar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SubGuitar;
import frc.robot.subsystems.SubSwitchPanel;
import frc.robot.subsystems.SubCandle;


public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  private SubGuitar s_Guitar;
  private SubCandle s_Candle;
  private SubSwitchPanel s_SwitchPanel;
  int iCount = 0;

  public IntakeNote(SubGuitar s_Guitar, SubCandle s_Candle, SubSwitchPanel s_SwitchPanel) {
    addRequirements(s_Guitar);
    this.s_Guitar = s_Guitar;
    this.s_Candle = s_Candle;
    this.s_SwitchPanel = s_SwitchPanel;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Guitar.runIntakeAtSpeed(Constants.guitar.intake.motor.dutyCycleIntake);

    if (iCount < 10) {
      // s_Candle.setLED_All(255, 70, 0);
      s_Candle.setLED_ToAlliance(s_SwitchPanel);
    } else {
      // s_Candle.setLED_NotCandle(0, 0, 0);
      s_Candle.setLED_NotCandle(255, 70, 0);
    }
    iCount++;
    iCount %= 20;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (s_Guitar.getIntakeHasNote()) {
      s_Candle.setLED_All(255, 70, 0);  //sets the LEDs to a nice orange color
    } else if (!s_Guitar.getIntakeHasNote()) {
      s_Candle.setLED_AllianceColorNoSwitch();
    }
    s_Guitar.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return s_Guitar.getIntakeHasNote();
  }
}