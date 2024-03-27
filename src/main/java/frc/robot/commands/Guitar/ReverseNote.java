// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Guitar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubCandle;
import frc.robot.subsystems.SubGuitar;
import frc.robot.subsystems.SubSwitchPanel;

public class ReverseNote extends Command {
  public SubGuitar s_Guitar;
  public SubCandle s_Candle;
  public SubSwitchPanel s_SwitchPanel;
  public double dUpperSpd;
  public double dLowerSpd;
  int iCount = 0;
  //This command reverses a note out of the intake and resets the LEDs to the Alliance color

  /** Creates a new ReverseNote. */
  public ReverseNote(SubGuitar s_Guitar, SubCandle s_Candle, SubSwitchPanel s_SwitchPanel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Guitar = s_Guitar;
    this.s_Candle = s_Candle;
    this.s_SwitchPanel = s_SwitchPanel;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Guitar.runIntakeAtSpeed(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!s_Guitar.getIntakeHasNote()) {
      s_Candle.setLED_ToAlliance(s_SwitchPanel);
    }
    s_Guitar.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!s_Guitar.getIntakeHasNote()) {
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
