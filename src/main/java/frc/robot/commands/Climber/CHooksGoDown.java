// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.climberLeft;
import frc.robot.Constants.climberRight;
import frc.robot.subsystems.SubClimber;

public class CHooksGoDown extends Command {
  private final SubClimber s_Climber;

  /** Creates a new GoHome. */
  public CHooksGoDown(SubClimber s_Climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
    this.s_Climber = s_Climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /** Runs the hooks down until they reach the home position
  */
  @Override
  public void execute() {
    if (s_Climber.getClimberLeftHookIsAtMinHeight() && s_Climber.getClimberRightHookIsAtMinHeight()) {
      s_Climber.runClimberLeftHookAtSpeed(0);
      s_Climber.runClimberRightHookAtSpeed(0);
    } else if (!s_Climber.getClimberLeftHookIsAtMinHeight() && !s_Climber.getClimberRightHookIsAtMinHeight()) {
      s_Climber.runClimberLeftHookAtSpeed(climberLeft.hook.motor.dutyCycleToHome);
      s_Climber.runClimberRightHookAtSpeed(climberRight.hook.motor.dutyCycleToHome);
    } else if (s_Climber.getClimberLeftHookIsAtMinHeight() && !s_Climber.getClimberRightHookIsAtMinHeight()) {
      s_Climber.runClimberLeftHookAtSpeed(0);
      s_Climber.runClimberRightHookAtSpeed(climberRight.hook.motor.dutyCycleToHome);
    } else if (!s_Climber.getClimberLeftHookIsAtMinHeight() && s_Climber.getClimberRightHookIsAtMinHeight()) {
      s_Climber.runClimberLeftHookAtSpeed(climberLeft.hook.motor.dutyCycleToHome);
      s_Climber.runClimberRightHookAtSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Climber.runClimberLeftHookAtSpeed(0);
    s_Climber.runClimberRightHookAtSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Climber.getClimberLeftHookIsAtMinHeight() && s_Climber.getClimberRightHookIsAtMinHeight());
  }
}
