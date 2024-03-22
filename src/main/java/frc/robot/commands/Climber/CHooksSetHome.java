// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.climberLeft;
import frc.robot.Constants.climberRight;
import frc.robot.subsystems.SubClimber;

public class CHooksSetHome extends Command {
  private final SubClimber s_Climber;
  /** Creates a new SetHome. */
  public CHooksSetHome(SubClimber s_Climber) {
    this.s_Climber = s_Climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Climber.setClimberLeftHookHome(climberLeft.hook.motor.posDownFull);
    s_Climber.setClimberRightHookHome(climberRight.hook.motor.posDownFull);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
