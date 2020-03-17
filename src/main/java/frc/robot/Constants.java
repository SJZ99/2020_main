/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class MotorConstants{
        /**
         * motor constants
         */
        public static final int krightmotor = 18;
        public static final int krightmotorS = 20;
        public static final int kleftmotor = 19;
        public static final int kleftmotorS = 21;
        /**
         * Invert or not
         */
        public static final boolean isRightMotorInvert = true;
        public static final boolean isLeftMotorInvert = false;
        public static final boolean isRightPhaseInvert = true;
        public static final boolean isLeftPhaseInvert = false;
        /**
         * distance per pulse
         */
        public static final double distantsPerPulse = 0.1524 * Math.PI / 2048 / 9.7;
         
    }
    public final class TrajectoryFoller{
        /**
         * PID constants
         */
        public static final double kS = 0.143;
        public static final double kV = 2.23;
        public static final double kA = 0.372;
        public static final double kP = 1.5;
        public static final double kI = 0;
        public static final double kD = 0;
        /**
         * wheel constants
         */
        public static final double wheelDiamete = 0.1524;
        public static final double wheelPitch = 0.7407;
    } 
}
