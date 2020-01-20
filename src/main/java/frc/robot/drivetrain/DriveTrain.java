package frc.robot.drivetrain;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * Class that runs the drive train
 * 
 * <p>
 * Autonomous:
 * <ul>
 * <li>update: Resets variables every autonomous cycle before running spinUp or
 * spinDown.</li>
 * <li>dashboardRun: Update SmartDashboard with the new variables</li>
 * <li>goToPose: Tries to move the robot to the give pose</li>
 * <li>interruptTrajectory: Interrupts current trajectory and stops motors</li>
 * </ul>
 * <p>
 * Operation: The left and right joystick on the drive controller will control
 * the left and right motors of the drive train respectively.
 * </p>
 * 
 * Note: teleopRun() will try to get driveExponent from the SmartDashboard and
 * will default to 1.8 if not found.
 * 
 * @author Ryan Chaiyakul
 */
public class DriveTrain {
    private XboxController driveController; // Controller checked during run()
    private CANSparkMax lsparkA, lsparkB, lsparkC, rsparkA, rsparkB, rsparkC; // Neos for the driveTrain
    private ArrayList<CANSparkMax> sparkList; // List of Neos from L to R and A to C
    private SpeedControllerGroup leftGroup, rightGroup; // SpeedController groups for each side
    private DifferentialDrive drive; // DifferentialDrive object
    private double slowPower, regularPower, turboPower; // Save power values

    private Kinematics kinematics; // Kinematics object for driveTrain
    private DriveController controller; // Controller for trajectory following

    private FollowTrajectory followTrajectory; // Trajectory generation and returns speeds to follow course
    private double trajectoryTime; // Time to finish trajectory

    public DriveTrain(int[] leftPorts, int[] rightPorts, int maxCurrent, double slowPower, double regularPower,
            double turboPower, XboxController driveController) {
        // 3 Ports for each side
        if (leftPorts.length != 3 || rightPorts.length != 3) {
            return;
        }

        lsparkA = new CANSparkMax(leftPorts[0], MotorType.kBrushless);
        lsparkB = new CANSparkMax(leftPorts[1], MotorType.kBrushless);
        lsparkC = new CANSparkMax(leftPorts[2], MotorType.kBrushless);

        rsparkA = new CANSparkMax(rightPorts[0], MotorType.kBrushless);
        rsparkB = new CANSparkMax(rightPorts[1], MotorType.kBrushless);
        rsparkC = new CANSparkMax(rightPorts[2], MotorType.kBrushless);

        // Group sparks into an ArrayList for a cleaner intialization loop
        sparkList = new ArrayList<CANSparkMax>() {
            {
                add(lsparkA);
                add(lsparkB);
                add(lsparkC);
                add(rsparkA);
                add(rsparkB);
                add(rsparkC);
            }
        };

        for (CANSparkMax spark : sparkList) {
            spark.setSmartCurrentLimit(maxCurrent);
            spark.setSecondaryCurrentLimit(maxCurrent);
            spark.setIdleMode(IdleMode.kBrake); // IdleMode will always be kBrake for driveTrain motors
        }

        leftGroup = new SpeedControllerGroup(lsparkA, lsparkB, lsparkC);
        rightGroup = new SpeedControllerGroup(rsparkA, rsparkB, rsparkC);

        drive = new DifferentialDrive(leftGroup, rightGroup);

        // Store variables
        this.driveController = driveController;
        this.slowPower = slowPower;
        this.regularPower = regularPower;
        this.turboPower = turboPower;
    }

    // ? Should the pose be relative to the center or one of the corners
    /**
     * Function to create Kinematics object
     * 
     * @param trackWidth   constant found from emperical data.
     *                     https://github.com/wpilibsuite/frc-characterization
     * @param startingPose cuurent Pose2d relative to the center of the field
     */
    public void enableKinematics(double trackWidth, double wheelRadius, Pose2d startingPose) {
        kinematics = new Kinematics(wheelRadius, lsparkA.getEncoder(), rsparkA.getEncoder(), trackWidth, startingPose);
    }

    /**
     * Function to create driveController object
     * 
     * @param maxOutput absolute value maximum voltage output [0,1]
     * @param kP        constant for P in PID
     */
    public void enableDriveController(double maxOutput, double kP) {
        controller = new DriveController(lsparkA.getEncoder(), rsparkA.getEncoder(), lsparkA.getPIDController(),
                rsparkA.getPIDController(), maxOutput, kP);
    }

    /**
     * Function to create driveController object
     * 
     * @param maxOutput absolute value maximum voltage output [0,1]
     * @param kP        constant for P in PID
     * @param kI        constant for I in PID
     */
    public void enableDriveController(double maxOutput, double kP, double kI) {
        controller = new DriveController(lsparkA.getEncoder(), rsparkA.getEncoder(), lsparkA.getPIDController(),
                rsparkA.getPIDController(), maxOutput, kP, kI);
    }

    /**
     * 
     * Function to create driveController object
     * 
     * @param maxOutput absolute value maximum voltage output [0,1]
     * @param kP        constant for P in PID
     * @param kI        constant for I in PID
     * @param kD        constant for D in PID
     */
    public void enableDriveController(double maxOutput, double kP, double kI, double kD) {
        controller = new DriveController(lsparkA.getEncoder(), rsparkA.getEncoder(), lsparkA.getPIDController(),
                rsparkA.getPIDController(), maxOutput, kP, kI, kD);
    }

    /**
     * Function to create Trajectory object
     * 
     * @param b    contant for B in Ramesete
     * @param zeta constant for Zeta in Ramsete
     */
    public void enableFollowTrajectory(double b, double zeta) {
        followTrajectory = new FollowTrajectory(kinematics.getKinematics());
        followTrajectory.enableRamseteController(b, zeta);
    }

    /**
     * Function that should be called by teleopPeriodic
     */
    public void run() {
        followTrajectory.run();
        kinematics.run();

        if (driveController.getAButton()) {
            return;
        } else {
            interruptTrajectory();
            runTankDrive();
        }
        dashboardRun();
    }

    /**
     * Function that updates SmartDashboard and variables
     */
    public void update() {
        followTrajectory.run();
        kinematics.run();
    }

    /**
     * Go to pose from current pose of the robot (Uses cubic hermite)
     * 
     * @param endPose         target pose
     * @param maxVelocity     in m/s
     * @param maxAcceleration in m/s^2
     */
    public void goToPose(Pose2d endPose, double maxVelocity, double maxAcceleration) {
        goToPose(endPose, new TrajectoryConfig(maxVelocity, maxAcceleration));
    }

    /**
     * Go to pose from current pose of the robot (Uses cubic hermite)
     * 
     * @param endPose          target pose
     * @param TrajectoryConfig
     */
    public void goToPose(Pose2d endPose, TrajectoryConfig trajectoryConfig) {
        goToPose(endPose, new ArrayList<Translation2d>() {
        }, trajectoryConfig);
    }

    /**
     * Go to pose from current pose of the robot (Uses cubic hermite)
     * 
     * @param endPose          target pose
     * @param poseList         in between poses
     * @param TrajectoryConfig
     */
    public void goToPose(Pose2d endPose, ArrayList<Translation2d> poseList, TrajectoryConfig trajectoryConfig) {
        if (!controller.isEnabled()) {
            return;
        }

        if (trajectoryTime == 0) {
            followTrajectory.setTrajectory(
                    TrajectoryGenerator.generateTrajectory(kinematics.getPose(), poseList, endPose, trajectoryConfig));
            trajectoryTime = followTrajectory.getTime();
        }

        DifferentialDriveWheelSpeeds wheelSpeed = followTrajectory.followTrajectory(kinematics.getPose());
        // wheelSpeed is null if trajectory is not intialized correctly
        if (wheelSpeed.equals(null)) {
            return;
        }
        setWheelSpeed(wheelSpeed);
    }

    /**
     * Interrupts current trajectory
     */
    public void interruptTrajectory() {
        followTrajectory.resetTrajectory();
        trajectoryTime = 0;

        leftGroup.set(0);
        rightGroup.set(0);
    }

    /**
     * Utilizes PID to try and maintain speed to complete the trajectory
     * 
     * @param newWheelSpeed
     */
    private void setWheelSpeed(DifferentialDriveWheelSpeeds newWheelSpeed) {
        controller.setLVelocity(kinematics.getWheelRPM('l', newWheelSpeed));
        controller.setRVelocity(kinematics.getWheelRPM('r', newWheelSpeed));

        leftGroup.set(lsparkA.get());
        rightGroup.set(rsparkA.get());
    }

    /**
     * Function that sets the speeds for the DifferentialDrive object periodically
     */
    private void runTankDrive() {
        // constants to easily configure if drive is opposite
        int constR = 1, constL = 1;

        // Get vertical value of the joysticks
        double rAxis = driveController.getY(Hand.kRight);
        double lAxis = driveController.getY(Hand.kLeft);

        // Use a constant multiplier for +/- direction as the driveExponent could be
        // even and negate the sign
        if (rAxis < 0) {
            constR *= 1;
        } else if (rAxis > 0) {
            constR *= -1;
        }

        if (lAxis < 0) {
            constL *= 1;
        } else if (lAxis > 0) {
            constL *= -1;
        }

        // LB and RB are used to change the drivePower on the fly
        double drivePower = regularPower;
        if (driveController.getBumper(Hand.kLeft))
            drivePower = slowPower;
        else if (driveController.getBumper(Hand.kRight))
            drivePower = turboPower;

        // However driveExponent should be constant (Changable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("driveExponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        double driveL = constL * drivePower * Math.pow(Math.abs(lAxis), driveExponent);
        double driveR = constR * drivePower * Math.pow(Math.abs(rAxis), driveExponent);

        // Our drivers prefer tankDrive
        drive.tankDrive(driveL, driveR);
    }

    /**
     * Function that should be called before autonomousPeriodic ends in autonomous.
     * Only exposed for autonomous in order to insure continually updates to
     * SmartDashboard.
     */
    public void dashboardRun() {
        double[] motorTemps = new double[6];
        for (int i = 0; i < motorTemps.length; i++) {
            motorTemps[i] = sparkList.get(i).getMotorTemperature();
        }

        SmartDashboard.putNumberArray("DriveTrainTemperature(lA|lB|lC|rA|rB|rC)", motorTemps);
    }
}