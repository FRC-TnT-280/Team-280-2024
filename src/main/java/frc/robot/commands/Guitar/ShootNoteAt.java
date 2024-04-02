// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Guitar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.guitar.shooterLower;
import frc.robot.Constants.guitar.shooterUpper;
import frc.robot.Constants.guitar;
import frc.robot.Constants.guitar.intake;
import frc.robot.subsystems.SubCandle;
import frc.robot.subsystems.SubGuitar;
import frc.robot.subsystems.SubSwitchPanel;
import edu.wpi.first.wpilibj.Timer;

public class ShootNoteAt extends Command {
  public SubGuitar s_Guitar;
  public SubCandle s_Candle;
  public SubSwitchPanel s_SwitchPanel;
  public double dUpperSpd;
  public double dLowerSpd;
  private int iCount = 0;

  /** Creates a new ShootNoteAt. */
  public ShootNoteAt(SubGuitar s_Guitar, SubCandle s_Candle, SubSwitchPanel s_SwitchPanel, double dUpperSpd, double dLowerSpd) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_Guitar);
    this.s_Guitar = s_Guitar;
    this.s_Candle = s_Candle;
    this.s_SwitchPanel = s_SwitchPanel;
    this.dUpperSpd = dUpperSpd;
    this.dLowerSpd = dLowerSpd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Guitar.runShooterLowerAtSpeed(dLowerSpd);
    s_Guitar.runShooterUpperAtSpeed(dUpperSpd);
    if (MathUtil.isNear(dLowerSpd, s_Guitar.getShooterLowerSpeed(), shooterLower.motor.velTolerance) 
    && MathUtil.isNear(dUpperSpd, s_Guitar.getShooterUpperSpeed(), shooterUpper.motor.velTolerance)
    /* && MathUtil.isNear(s_Guitar.getGuitarPivotPosition(), Constants.guitar.pivot.posMid, 0.001) */) {
      iCount++;
    }

    if (iCount >= guitar.SHOOTER_AT_SPEED_DEBOUNCE) {
      s_Guitar.runIntakeAtSpeed(intake.motor.dutyCycleShoot);
      iCount = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (s_SwitchPanel.getAllianceBlueSwitch()) {
    //   s_Candle.setLED_NotCandle(0, 0, 255);
    // } else if (s_SwitchPanel.getAllianceRedSwitch()) {
    //   s_Candle.setLED_NotCandle(255,0,0);
    // }
    s_Candle.setLED_ToAlliance(s_SwitchPanel);
    s_Guitar.stopShooter();
    s_Guitar.stopIntake(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_Guitar.getIntakeHasNote();
  }
}
