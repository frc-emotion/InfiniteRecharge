package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Constants {
    /**
     * Controller Ports
     */
    static final int kDrivePort = 0;
    static final int kOperatorPort = 1;

    /**
     * USB Ports
     */
    static final Port kGyroPort = Port.kUSB;

    /**
     * CANBUS Ports
     */
    static final int[] kShooterPorts = { 0, 1 };
    static final int kShooterForwardPort = 2;
    static final int kShooterReversePort = 3;
    static final int[] kDriveLeftPorts = { 3, 4, 5 };
    static final int[] kDriveRightPorts = { 9, 10, 11 };
    static final int kPivotPort = 4;
    static final int[] kIntakePorts = { 5, 6 };

    /**
     * Digital Ports
     */

    static final int kPivotLowerLimitPort = 0;
    static final int kPivotUpperLimitPort = 1;

    /**
     * Robot Constants
     */

    // Flipper Constants

    static final double kTrackWidth = 0.6898513;
    static final double kWheelRadius = 0.15242;
    static final double kMaxVelocity = 2;
    static final double kMaxAcceleration = 0.5; // Not verified

    /**
     * Motor Defaults
     */
    static final int kSparkMaxCurrent = 35;

    /**
     * Pivot Constants
     */

    static final double kTeleopConstant = 0.5;
    static final double kCallibrateSpeed = -0.1;
    static final double kRevToAngle = 42;
    static final double kPivotThreshold = 0.3;
    static final double kPivotMaxAngle = 20;

    /**
     * Intake Constants 
     */

     static final double kIntakeThreshold = 0.3;
     static final double kIntakeMaxOutput = 0.5;

    /**
     * Shooter General Constants
     */
    static final double kShooterRPM = 300;
    static final double kShooterThresholdRPM = 30;
    static final double kShooterThresholdTrigger = 0.3;
    static final double kShooterMaxOutput = 0.7;
    static final double kShooterWaitTime = 500;

    /**
     * Shooter Choosable Controller Constants
     */
    static final String kTwoStep = "TwoStep";
    static final String kPID = "PID";

    /**
     * Shooter Two Step Constants
     */
    static final double kShooterMinOutput = 0;

    /**
     * Shooter PID Constants
     */
    static final float kPShooter = 1;
    static final float kIShooter = 0;
    static final float kDShooter = 0;

    /**
     * DriveTrain General Constants
     */
    static final double kSlowPower = 0.4;
    static final double kRegularPower = 0.7;
    static final double kTurboPower = 0.99;

    /**
     * DriveTrain PID Constants
     */
    static final double kPDrive = 1;
    static final double kIDrive = 0;
    static final double kDDrive = 0;

    /**
     * Pose2d Constants All Pose2d should be relative to one Pose2d (Currently
     * center of the field)
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