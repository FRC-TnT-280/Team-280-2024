// package frc.lib.util.auton;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;

// import frc.robot.Constants;

// /*Returns each trajectory used in auton */
// public final class Trajectories {
//     //test Trajectories
//     public static PathPlannerTrajectory testTrajectory1() {
//         return PathPlanner.loadPath(
//                 "testTrajectory1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory testTrajectory2() {
//         return PathPlanner.loadPath(
//                 "testTrajectory2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     //Blue Trajectories
//     public static PathPlannerTrajectory BlueCone6_1() {
//         return PathPlanner.loadPath(
//                 "BlueCone6_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone6_2() {
//         return PathPlanner.loadPath(
//                 "BlueCone6_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone5_1() {
//         return PathPlanner.loadPath(
//                 "BlueCone5_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     public static PathPlannerTrajectory BlueCone5_2() {
//         return PathPlanner.loadPath(
//                 "BlueCone5_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone5_3() {
//         return PathPlanner.loadPath(
//                 "BlueCone5_3",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone5_4() {
//         return PathPlanner.loadPath(
//                 "BlueCone5_4",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     public static PathPlannerTrajectory BlueCone4Charge_1() {
//         return PathPlanner.loadPath(
//                 "BlueCone4Charge_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone4Charge_2() {
//         return PathPlanner.loadPath(
//                 "BlueCone4Charge_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone4Charge_3() {
//         return PathPlanner.loadPath(
//                 "BlueCone4Charge_3",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCone4Charge_4() {
//         return PathPlanner.loadPath(
//                 "BlueCone4Charge_4",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }      
    
//     public static PathPlannerTrajectory BlueCube3_1() {
//         return PathPlanner.loadPath(
//                 "BlueCube3_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory BlueCube3_2() {
//         return PathPlanner.loadPath(
//                 "BlueCube3_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }
    
//     public static PathPlannerTrajectory BlueCube3_3() {
//         return PathPlanner.loadPath(
//                 "BlueCube3_3",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     public static PathPlannerTrajectory BlueCube3_4() {
//         return PathPlanner.loadPath(
//                 "BlueCube3_4",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     //Red Trajectories
//     public static PathPlannerTrajectory RedCone6_1() {
//         return PathPlanner.loadPath(
//                 "RedCone6_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone6_2() {
//         return PathPlanner.loadPath(
//                 "RedCone6_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone5_1() {
//         return PathPlanner.loadPath(
//                 "RedCone5_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     public static PathPlannerTrajectory RedCone5_2() {
//         return PathPlanner.loadPath(
//                 "RedCone5_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone5_3() {
//         return PathPlanner.loadPath(
//                 "RedCone5_3",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone5_4() {
//         return PathPlanner.loadPath(
//                 "RedCone5_4",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     public static PathPlannerTrajectory RedCone4Charge_1() {
//         return PathPlanner.loadPath(
//                 "RedCone4Charge_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone4Charge_2() {
//         return PathPlanner.loadPath(
//                 "RedCone4Charge_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone4Charge_3() {
//         return PathPlanner.loadPath(
//                 "RedCone4Charge_3",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCone4Charge_4() {
//         return PathPlanner.loadPath(
//                 "RedCone4Charge_4",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }      
    
//     public static PathPlannerTrajectory RedCube3_1() {
//         return PathPlanner.loadPath(
//                 "RedCube3_1",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     public static PathPlannerTrajectory RedCube3_2() {
//         return PathPlanner.loadPath(
//                 "RedCube3_2",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }
    
//     public static PathPlannerTrajectory RedCube3_3() {
//         return PathPlanner.loadPath(
//                 "RedCube3_3",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }   
    
//     public static PathPlannerTrajectory RedCube3_4() {
//         return PathPlanner.loadPath(
//                 "RedCube3_4",
//                 Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//                 Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     }

//     // public static PathPlannerTrajectory ThreeCubePurpleCannon_1() {
//     //     return PathPlanner.loadPath(
//     //             "ThreeCubePurpleCannon_Part1",
//     //             Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//     //             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
//     // }    
//     // public static PathPlannerTrajectory BalanceFromFreelane() {
//     //     return PathPlanner.loadPath(
//     //             "BalanceFromFreelane",
//     //             Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
//     //             Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
//     // }

    

// }
