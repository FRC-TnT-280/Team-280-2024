// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Guitar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.guitar.pivot;
import frc.robot.subsystems.SubGuitar;

public class GPivotGoTo extends Command {
  private final SubGuitar s_Guitar;
  private final double d_targetPos;

  /** Creates a new GPivotGoTo. */
  public GPivotGoTo(SubGuitar s_Guitar, double d_targetPos) {
    addRequirements(s_Guitar);
    this.s_Guitar = s_Guitar;
    this.d_targetPos = d_targetPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Guitar.movePivotLeftToPosition(d_targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if ((d_targetPos == pivot.posHome) || (d_targetPos == pivot.posTrap)) {
      s_Guitar.runPivotAtSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (MathUtil.isNear(d_targetPos, s_Guitar.getGuitarPivotPosition(), pivot.positionTolerance)) {
      return true;
    } else {
      return false;
    }
  }
}
