// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.lib.util.auton;

// //import java.util.HashMap;
// import com.PathPlanner.lib.PathPlannerTrajectory;
// //import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.controller.PIDController;
// //import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystem.SubSwerve;

// /** Add your docs here. */
// public class AutonUtils {

//     public static PPSwerveControllerCommand getSwerveControllerCommand(SubSwerve swerveInstance, PathPlannerTrajectory traj){
        
//         return new PPSwerveControllerCommand(
//             traj, 
//             swerveInstance::getPose,
//             Constants.Swerve.swerveKinematics, 
//             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//             new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
//             new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
//             swerveInstance::setModuleStates, 
//             false
//         );
//     }

//     // public static FollowPathWithEvents getPathWithEvents(PathPlannerTrajectory traj, HashMap<String, Command> eventMap){
//     //     return new FollowPathWithEvents(getSwerveControllerCommand(null, traj), traj.getMarkers(), eventMap);
//     // }
// }
