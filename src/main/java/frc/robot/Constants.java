package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Constants {
    /**
     * USB Ports
     */
    static final int kDrivePort = 0;
    static final int kOperatorPort = 1;

    /**
     * CANBUS Ports
     */
    static final int[] kShooterPorts = { 0, 1 };
    static final int[] kDriveLeftPorts = { 3, 4, 5 };
    static final int[] kDriveRightPorts = { 9, 10, 11 };

    /**
     * Robot Constants
     */
    static final double kTrackWidth = 0.2; // in meters

    /**
     * Motor Defaults
     */
    static final int kSparkMaxCurrent = 35;

    /**
     * Shooter General Constants
     */
    static final double kShooterRPM = 300;
    static final double kShooterThreshold = 30;
    static final double kShooterMaxOutput = 0.5;

    /**
     * Shooter Choosable Controller Constants
     */
    static final String kTwoStep = "TwoStep";
    static final String kPID = "PID";

    /**
     * Two Step Constants
     */
    static final double kShooterMinOutput = 0.3;

    /**
     * Shooter PID Constants
     */
    static final float kPShooter = 1 / 300;
    static final float kIShooter = 0;
    static final float kDShooter = 0;

    /**
     * DriveTrain General Constants
     */
    static final double kSlowPower = 0.4;
    static final double kRegularPower = 0.7;
    static final double kTurboPower = 0.99;

    /**
     * Pose2d Constants
     * All Pose2d should be relative to one Pose2d (Currently center of the field)
     */
    static final ArrayList<ArrayList<Pose2d>> kStartingLocations = new ArrayList<ArrayList<Pose2d>>() {
        {
            // Blue Alliance
            add(new ArrayList<Pose2d>() {
                {
                    add(new Pose2d());
                    add(new Pose2d());
                    add(new Pose2d());
                }

            });
            // Red Alliance
            add(new ArrayList<Pose2d>() {
                {
                    add(new Pose2d());
                    add(new Pose2d());
                    add(new Pose2d());
                }

            });
        }

    };

    /**
     * Autonomous Constants
     */
    static final ArrayList<Character> kValidColors = new ArrayList<Character>() {
        {
            add('B');
            add('G');
            add('R');
            add('Y');
        }
    };
}