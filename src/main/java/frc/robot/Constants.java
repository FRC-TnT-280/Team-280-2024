// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    /** Expansion Slot Digital IO */
    public static final class expansionIO {
        public static final int rotary01 = 11;
        public static final int rotary02 = 12;
        public static final int rotary03 = 13;
        public static final int rotary04 = 14;
        public static final int rotary05 = 15;
        public static final int rotary06 = 16;
        public static final int rotary07 = 17;
        public static final int rotary08 = 18;
        public static final int rotary09 = 19;
        public static final int rotary10 = 20;
        public static final int rotary11 = 21;
        public static final int rotary12 = 22;
        public static final int allianceBlue = 24;
        public static final int allianceRed = 25;
        public static final int setupMode = 23;
        // public static final int spare01 = 10;
    }

    /** Driver & operator controller constants */
    public static final double stickDeadband = 0.1;

    /** CANdle constants */
    public static final class candle {
        public static final int canID = 40;
    }
   
    /** Climber left constants */
    public static final class climberLeft {
        /** Climber left pivot constants */
        public static final class pivot { 
            public static final class motor {
                public static final int canID = 30;
                public static final double kP = 1.0;
                public static final double kI = 0.25;
                public static final double kD = 0.001;
                public static final boolean bInverted = false; // invert
            }

            public static final class encoder {
                public static final int canID = 31; 
                public static final boolean bInverted = false; // invert
                public static final double dMagnetOffset = -0.23584; 
            }

            public static final double posHome = 0; // climber pivot home position
            public static final double posClimb = 0.18; // climber pivot climb position
            public static final double posTolerance = 0.001; // tolerance for pivot in position
        }

        /** Climber left hook constants */
        public static final class hook {
            public static final class motor {
                public static final int canID = 32;
                public static final double kP = 0.1;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final boolean bInverted = false; // invert
                public static final double dutyCycleToClimb = 0.9; // hook up speed
                public static final double dutyCycleToHome = -0.9; // hook down speed
                public static final double posDownFull = 0; // hook full retract position
                public static final double posUpFull = 350; // hook full extend position //TODO set hook full up position
                public static final double posTolerance = 5; // tolerance for hook in position
            }
        }
    }

    /** Climber right constants */
    public static final class climberRight {
        /** Climber right pivot constants */
        public static final class pivot {
            public static final class motor {
                public static final int canID = 33;
                public static final double kP = 1.0;
                public static final double kI = 0.25;
                public static final double kD = 0.001;
                public static final boolean bInverted = true; // invert
            }

            public static final class encoder {
                public static final int canID = 34; 
                public static final boolean bInverted = true; // invert
                public static final double dMagnetOffset = -0.376953125;
            }

            public static final double posHome = 0; // climber pivot home position
            public static final double posClimb = 0.18; // climber pivot climb position
            public static final double posTolerance = 0.001; // tolerance for pivot in position
        }

        /** Climber hook right constants */
        public static final class hook {
            public static final class motor {
                public static final int canID = 35;
                public static final double kP = 0.2;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final boolean bInverted = false; // invert
                public static final double dutyCycleToClimb = .9; // hook up speed
                public static final double dutyCycleToHome = -.9; // hook down speed
                public static final double posDownFull = 0; // hook full retract position
                public static final double posUpFull = 350; // hook full extend position //TODO set hook full up position
                public static final double posTolerance = 5; // tolerance for hook in position
            }
        }
    }

    public static final class guitar {
        /** Guitar intake roller constants */
        public static final class intake {
            public static final class motor {
                public static final int canID = 14;
                public static final boolean bInverted = false; //invert
                public static final double dutyCycleIntake = .6; // intake pickup speed
                public static final double dutyCycleShoot = 1; // intake shoot speed
            }
        
            public static final int ioNoteInPlace = 6; // photoeye for note in place -- normally closed
        }

        /** Shooter upper roller constants */
        public static final class shooterUpper {
            public static final class motor {
                public static final int canID = 20;
                public static final double kP = .05; // 0.059
                public static final double kI = 0.015; // 0.2
                public static final double kD = 0.0; // 0
                public static final double kV = .01; 
                public static final boolean bInverted = true; // invert
                public static final double velTolerance = .5; // window for determining when shooter is at speed
                public static final double velDefaultRange = 54; //52; // RPS for shooting at the default range
                public static final double velClearNote = 35; // just enough to get over the bot for denials
            }
        }
        
        /** Shooter lower roller constants */
        public static final class shooterLower {
            public static final class motor {
                public static final int canID = 21;
                public static final double kP = .05; // 0.059
                public static final double kI = 0.0; // 0.1
                public static final double kD = 0.0; // 0.00015
                public static final double kV = .01;
                public static final boolean bInverted = true; // invert
                public static final double velTolerance = .5; // window for determining when shooter is at speed
                public static final double velDefaultRange = 30; //27; // RPS for shooting at the default range
                public static final double velClearNote = 35; // just enough to get over the bot for denials
            }

        }
        
        /** Guitar pivot constants */
        public static final class pivot {
            public static final class motorLeft {
                public static final int canID = 22;
                public static final double kP = 2.4; // 2.2
                public static final double kI = 1.2; // 0.1
                public static final double kD = 0.0; // 0.1
                public static final double kG = 0.02; // 0
                public static final boolean bInverted = false; // invert
            }

            public static final class motorRight {
                public static final int canID = 23;
                public static final double kP = 2.2;
                public static final double kI = 0.1;
                public static final double kD = 0.1;
                // public static final double kG = 0.1;
                public static final boolean bInverted = true; //invert
            }

            public static final class encoder {
                public static final int canID = 24;
                public static final boolean bInverted = false; // invert
                public static final double dMagnetOffset = -0.43212890625; // sets the offset between the absolute position and actual position
            }

            public static final double posAmp = 0.25; // sets the position of the pivot for scoring in the amp
            public static final double posTrap = 0.3; // sets the position for scoring in the trap
            public static final double posHome = 0.006; // sets the home position / position of the pivot for pickup off the floor
            public static final double posMid = 0.077; // sets the middle, manual position of the pivot for shooting at a shallower angle
            public static final double positionTolerance = 0.001; // sets the position tolerance of the pivot

            public static final int ioHome = 7; // guitar pivot home limit switch -- normally closed
        }
    }

    public static final class lights {

    }
}
