package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that handles the kinematics and odometry for the differential
 * drivetrain
 * 
 * <p>
 * Operation: Call run periodically throughout autonomous and teleop. Call get
 * functions whenever needed.
 * </p>
 * 
 * Note: getWheelRPM has different function signatures.
 * 
 * @author Ryan Chaiyakul
 */
class Kinematics {
    private CANEncoder lEncoder, rEncoder; // Respective sparkA's encoder
    private AHRS gyro; // Gyro for heading

    private DifferentialDriveKinematics driveKinematics; // For autonomous and getting chassis information
    private DifferentialDriveOdometry driveOdometry; // To track the robot's location throughout the match

    private ChassisSpeeds currentSpeed; // Current chassis speed of the robot
    private Pose2d currentPose; // Current location and heading

    private double wheelRadius; // Radius of the driveTrain wheels

    Kinematics(double wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    Kinematics(Port gyroPort, double wheelRadius, CANEncoder lEncoder, CANEncoder rEncoder, double trackWidth, Pose2d startingPose) {
        this.lEncoder = lEncoder;
        this.rEncoder = rEncoder;

        gyro = new AHRS(gyroPort);

        reset(trackWidth, startingPose);

        this.wheelRadius = wheelRadius;
    }

    /**
     * Function to reset kinematics and odometry with new variables
     * 
     * @param trackWidth   constant found from emperical data.
     *                     https://github.com/wpilibsuite/frc-characterization
     * @param startingPose cuurent Pose2d relative to the center of the field
     */
    public void reset(double trackWidth, Pose2d startingPose) {
        resetSensors();

        driveKinematics = new DifferentialDriveKinematics(trackWidth);
        driveOdometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getAngle()), startingPose);
    }

    /**
     * Private function to reset the values of encoders and gyro
     */
    private void resetSensors() {
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);

        gyro.reset();
    }

    /**
     * Get differentialDriveKinematics object
     */
    public DifferentialDriveKinematics getKinematics() {
        return driveKinematics;
    }

    /**
     * Get the robots current Pose
     * 
     * @return Pose2d
     */
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * Get the selected wheel's current RPM
     * 
     * @param selector 'l' and 'r'
     * @return RPM of wheel
     */
    public double getWheelRPM(char selector) {
        return getWheelRPM(selector, currentSpeed);
    }

    /**
     * Get the selected wheel's current RPM from the provided chassisSpeed object
     * 
     * @param selector     'l' and 'r'
     * @param chassisSpeed
     * @return RPM of wheel
     */
    public double getWheelRPM(char selector, ChassisSpeeds chassisSpeed) {
        double linearSpeed;
        switch (selector) {
        case 'l':
            linearSpeed = driveKinematics.toWheelSpeeds(chassisSpeed).leftMetersPerSecond;
            break;
        case 'r':
            linearSpeed = driveKinematics.toWheelSpeeds(chassisSpeed).rightMetersPerSecond;
            break;
        default:
            linearSpeed = 0;
            break;
        }

        return linearSpeed * 60 / (wheelRadius * 2 * Math.PI);
    }

    /**
     * Get the selected wheel's current RPM from the provided wheelSpeed object
     * 
     * @param selector   'l' and 'r'
     * @param wheelSpeed
     * @return RPM of wheel
     */
    public double getWheelRPM(char selector, DifferentialDriveWheelSpeeds wheelSpeed) {
        double linearSpeed;
        switch (selector) {
        case 'l':
            linearSpeed = wheelSpeed.leftMetersPerSecond;
            break;
        case 'r':
            linearSpeed = wheelSpeed.rightMetersPerSecond;
            break;
        default:
            linearSpeed = 0;
            break;
        }

        return linearSpeed * 60 / (wheelRadius * 2 * Math.PI);
    }

    /**
     * Update kinematics and odometry
     */
    public void run() {
        int gyroConst = -1;

        DifferentialDriveWheelSpeeds wheelSpeed = new DifferentialDriveWheelSpeeds(
                lEncoder.getVelocity() * lEncoder.getVelocityConversionFactor(),
                rEncoder.getVelocity() * rEncoder.getVelocityConversionFactor());

        currentSpeed = driveKinematics.toChassisSpeeds(wheelSpeed);

        currentPose = driveOdometry.update(Rotation2d.fromDegrees(gyroConst * gyro.getAngle()),
                lEncoder.getPosition() * lEncoder.getPositionConversionFactor(),
                rEncoder.getPosition() * rEncoder.getPositionConversionFactor());

        SmartDashboard.putNumber("DriveTrainLinearVelocity", currentSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("DriveTrainAngularVelocity", currentSpeed.omegaRadiansPerSecond);
        SmartDashboard.putNumber("DriveTrainLeftWheelSpeed",
                driveKinematics.toWheelSpeeds(currentSpeed).leftMetersPerSecond);
        SmartDashboard.putNumber("DriveTrainLeftWheelSpeed",
                driveKinematics.toWheelSpeeds(currentSpeed).rightMetersPerSecond);
    }
}