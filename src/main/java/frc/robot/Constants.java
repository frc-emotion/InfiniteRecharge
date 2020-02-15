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
    static final int[] kShooterPorts = { 1, 2 };
    static final int kShooterForwardPort = 2;
    static final int kShooterReversePort = 3;
    static final int kIntakeForwardPort = 4;
    static final int kIntakeReversePort = 5;
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
     * LimeLight Pipelines
     */

     static final int kPortPipeline = 0;

    /**
     * Robot Constants
     */

    // Flipper Constants

    public static double WHEELBASE_WIDTH = 0.6898513;
    public static double WHEEL_DIAMETER = 0.1524;
    public static double MAX_VELOCITY = 2;
    static final double MAX_ACCELERATION = 0.5; // Not verified

    public static final int ENCODER_REV = 42;// temporary

    public static final int WHEEL_REV = 7;

    /**
     * LimeLight Constants
     */

     static final double kMountingHeight = 0.25;
     static final double kMountingAngle = 20;
     static final double kRefrenceHeight = 3.3;

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
    static final double kPivotControllerThreshold = 0.3;
    static final double kPivotAngleThreshold = 5;
    static final double kPivotMaxAngle = 20;

    /**
     * Intake Constants 
     */

     static final double kIntakeThreshold = 0.3;
     static final double kIntakeOutput = 0.5;
     static final double kTubeOutput = 0.5;

    /**
     * Shooter General Constants
     */
    static final double kShooterRPM = 5000;
    static final double kShooterThresholdRPM = 50;
    static final double kShooterThresholdTrigger = 0.3;
    static final double kShooterMaxOutput = 1;
    static final double kShooterWaitTime = 500;

    static final double kShooterMaxVelocity = 10;

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
    static final boolean kInvert = false; // default value

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