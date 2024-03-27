// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.climberLeft;
import frc.robot.Constants.climberRight;
import frc.robot.subsystems.SubClimber;

public class CPivotsGoTo extends Command {
  private final SubClimber s_Climber;
  private final double leftTarget;
  private final double rightTarget;

  /** Creates a new CPivotsGoTo. */
  public CPivotsGoTo(SubClimber s_Climber, double leftTarget, double rightTarget) {
    addRequirements(s_Climber);
    this.s_Climber = s_Climber;
    this.leftTarget = leftTarget;
    this.rightTarget = rightTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Climber.moveClimberLeftToPosition(leftTarget);
    s_Climber.moveClimberRightToPosition(rightTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (leftTarget == climberLeft.pivot.posClimb) {
      s_Climber.runClimberLeftAtSpeed(0);
    }
    if (rightTarget == climberRight.pivot.posClimb) {
      s_Climber.runClimberRightAtSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (MathUtil.isNear(leftTarget, s_Climber.getClimberLeftPivotPosition(), climberLeft.pivot.posTolerance) 
    && MathUtil.isNear(rightTarget, s_Climber.getClimberRightPivotPosition(), climberRight.pivot.posTolerance)) {
      return true;
    } else {
      return false;
    }
  }
}
