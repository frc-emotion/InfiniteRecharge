package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Todo Create a follow trajectory function
// TODO Create a goto Pose function
/**
 * Class that runs the drive train
 * 
 * <p>
 * Autonomous:
 * <ul>
 * <li>update: Resets variables every autonomous cycle before running spinUp or
 * spinDown.</li>
 * <li>dashboardRun: Update SmartDashboard with the new variables</li>
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
class DriveTrain {
    private XboxController driveController; // Controller checked during run()
    private CANSparkMax lsparkA, lsparkB, lsparkC, rsparkA, rsparkB, rsparkC; // Neos for the driveTrain
    private ArrayList<CANSparkMax> sparkList; // List of Neos from L to R and A to C
    private SpeedControllerGroup leftGroup, rightGroup; // SpeedController groups for each side
    private DifferentialDrive drive; // DifferentialDrive object

    private CANEncoder lEncoder, rEncoder; // Respective sparkA's encoder
    private AHRS gyro; // Gyro for heading

    private DifferentialDriveKinematics driveKinematics; // For autonomous and getting chassis information
    private DifferentialDriveOdometry driveOdometry; // To track the robot's location throughout the match

    private ChassisSpeeds currentSpeed; // Current chassis speed of the robot
    // ? Do we need to use the currentPose
    private Pose2d currentPose; // Current location and heading

    private double slowPower, regularPower, turboPower; // Save power values

    DriveTrain(int[] leftPorts, int[] rightPorts, int maxCurrent, double slowPower, double regularPower,
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

        // Initialize encoders
        lEncoder = lsparkA.getEncoder();
        rEncoder = rsparkA.getEncoder();

        // Initialize gyro
        gyro = new AHRS(Port.kUSB);

        resetSensors();

        // Store variables
        this.driveController = driveController;
        this.slowPower = slowPower;
        this.regularPower = regularPower;
        this.turboPower = turboPower;
    }

    DriveTrain(int[] leftPorts, int[] rightPorts, int maxCurrent, double slowPower, double regularPower,
            double turboPower, double trackWidth, Pose2d startingPose, XboxController driveController) {
        this(leftPorts, rightPorts, maxCurrent, slowPower, regularPower, turboPower, driveController);

        initializeKinematics(trackWidth, startingPose);
    }

    /**
     * Private function to reset the values of encoders and gyro
     */
    private void resetSensors() {
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);

        gyro.reset();
    }

    // ? Should the pose be relative to the center or one of the corners
    /**
     * Function to set kinematics and odometry
     * 
     * @param trackWidth   constant found from emperical data.
     *                     https://github.com/wpilibsuite/frc-characterization
     * @param startingPose cuurent Pose2d relative to the center of the field
     */
    public void initializeKinematics(double trackWidth, Pose2d startingPose) {
        resetSensors();

        driveKinematics = new DifferentialDriveKinematics(trackWidth);
        driveOdometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getAngle()), startingPose);
    }

    /**
     * Function that should be called by teleopPeriodic
     */
    public void run() {
        update();

        if (driveController.getAButton()) {

        } else {
            runTankDrive();
        }
        dashboardRun();
    }

    /**
     * Function that should be called periodically for autonomous. Only exposed for
     * autonomous in order to insure continually updates to SmartDashboard.
     */
    public void update() {
        int gyroConst = -1;

        DifferentialDriveWheelSpeeds wheelSpeed = new DifferentialDriveWheelSpeeds(
                lEncoder.getVelocity() * lEncoder.getVelocityConversionFactor(),
                rEncoder.getVelocity() * rEncoder.getVelocityConversionFactor());

        currentSpeed = driveKinematics.toChassisSpeeds(wheelSpeed);

        currentPose = driveOdometry.update(Rotation2d.fromDegrees(gyroConst * gyro.getAngle()),
                lEncoder.getPosition() * lEncoder.getPositionConversionFactor(),
                rEncoder.getPosition() * rEncoder.getPositionConversionFactor());
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
        SmartDashboard.putNumber("DriveTrainLinearVelocity", currentSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("DriveTrainAngularVelocity", currentSpeed.omegaRadiansPerSecond);
        SmartDashboard.putNumber("DriveTrainLeftWheelSpeed",
                driveKinematics.toWheelSpeeds(currentSpeed).leftMetersPerSecond);
        SmartDashboard.putNumber("DriveTrainLeftWheelSpeed",
                driveKinematics.toWheelSpeeds(currentSpeed).rightMetersPerSecond);

        double[] motorTemps = new double[6];
        for (int i = 0; i < motorTemps.length; i++) {
            motorTemps[i] = sparkList.get(i).getMotorTemperature();
        }

        SmartDashboard.putNumberArray("DriveTrainTemperature(lA|lB|lC|rA|rB|rC)", motorTemps);
    }
}