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
     * limit switch ports
     */
    static final int lowerClimbLimitPort = 1;
    static final int upperClimbLimitPort = 2;

    /**
     * USB Ports
     */
    static final Port kGyroPort = Port.kUSB;

    /**
     * CANBUS Ports
     */
    static final int[] kShooterPorts = { 1, 2 };
    static final int[] kDriveLeftPorts = { 3, 4, 5 };
    static final int[] kDriveRightPorts = { 6, 9, 10 };
    static final int kPivotPort = 11;
    static final int[] kIntakePorts = { 12, 13, 14, 15 };

    /**
     * Pneumatics Ports
     */
    static final int kShooterForwardPort = 1;
    static final int kShooterReversePort = 0;
    static final int kIntakeForwardPort = 4;
    static final int kIntakeReversePort = 5;

    /**
     * Digital Ports
     */

    static final int kPivotLowerLimitPort = 0;

    /**
     * LimeLight Pipelines
     */

    static final int kPortPipeline = 0;

    /**
     * LimeLight Constants
     */

    static final double kMountingHeight = 0.4191;
    static final double kMountingAngle = 40;
    static final double kRefrenceHeight = 0.249;

    /**
     * Motor Defaults
     */

    static final int kSparkMaxCurrent = 45;
    static final int kTalonMaxCurrent = 60; // bc 40 A breakers
    /**
     * Pivot Constants
     */

    static final double kTeleopConstant = 0.25;
    static final double kCallibrateSpeed = -0.1;
    static final double kRevToAngle = 42;
    static final double kPivotControllerThreshold = 0.3;
    static final double kPivotAngleThreshold = 5;
    static final double kPivotMaxAngle = 20;

    static final float kPivotP = 1;
    static final float kPivotI = 0;
    static final float kPivotD = 0;

    /**
     * Intake Constants
     */

    static final double kIntakeThreshold = 0.3;
    static final double kIntakeOutput = 0.3;
    static final double kTubeOutput = 0.5;

    /**
     * Shooter General Constants
     */
    static final double kShooterRPM = 5600;
    static final double kShooterThresholdRPM = 50;
    static final double kShooterThresholdTrigger = 0.3;
    static final double kShooterMaxOutput = 1;
    static final double kShooterWaitTime = 400;
    public static final double kShooterTurnSpeed = 0.2;
    public static final double kShooterRotateTime = 4000;

    public static final double kShooterVelocity = 10;
    /**
     * Shooter Choosable Controller Constants
     */
    static final String kTwoStep = "TwoStep";
    static final String kPID = "PID";

    /**
     * Shooter Two Step Constants
     */
    static final double kShooterMinOutput = 0.9;

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
    static final float kPDrive = 1;
    static final float kIDrive = 0;
    static final float kDDrive = 2.5f;
    static final double kDriveTolerance = 0.4;
    static final double kDriveRotation = 0.2;

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

    /**
     * Robot Constants
     */

    // Flipper Constants

    public static final int SCREW_TALON_L = 8;
    public static final int SCREW_TALON_R = 7;
    public static final double SCREW_SPEED = 0.975;

    // solenoids
    public static final int PISTONL_FWD = 2;
    public static final int PISTONL_BKWD = 3;

    public static double WHEELBASE_WIDTH = 0.6898513;
    public static double WHEEL_DIAMETER = 0.1524;
    public static double MAX_VELOCITY = 2;
    static final double MAX_ACCELERATION = 0.5; // Not verified

    public static final int ENCODER_REV = 42;// temporary

    public static final int WHEEL_REV = 7;
}