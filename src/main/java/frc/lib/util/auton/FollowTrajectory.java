// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.lib.util.auton;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystem.SubSwerve;

// public class FollowTrajectory extends Command {
//   private final SubSwerve drivetrain;
//   private final Timer timer = new Timer();
//   private final PathPlannerTrajectory trajectory;
//   private final PPHolonomicDriveController controller;
//   private PathPlannerState desiredState;
//   private ChassisSpeeds desiredChasisSpeeds;
//   private final boolean resetPose;

//   public FollowTrajectory(PathPlannerTrajectory trajectory, PPHolonomicDriveController controller, boolean resetPose, SubSwerve drivetrain) {
//     this.trajectory = trajectory;
//     this.controller = controller;
//     this.drivetrain = drivetrain;
//     this.resetPose = resetPose;
//     addRequirements(drivetrain);
//   }

//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();

//     if(resetPose) {
//       drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
//     }
//   }

//   @Override
//   public void execute() {
//     var pose = drivetrain.getPose();
//     desiredState = (PathPlannerState) trajectory.sample(timer.get());
//     desiredChasisSpeeds = this.controller.calculate(pose, desiredState);
//     drivetrain.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChasisSpeeds));
//   }

//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//   }

//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(trajectory.getTotalTimeSeconds());
//   }
// }
