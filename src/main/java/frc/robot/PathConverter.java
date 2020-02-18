package frc.robot;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.*;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Class that converts a PathFinder trajectory to robot motion.
 * <p>
 * Operation: Performed autnomously whenever needed. B is to pause, and any
 * stick movement quits it.
 * <p>
 * Notes: A helper class for DriveTrain.
 * 
 * @author Gokul Swaminathan
 */
public class PathConverter {

    // robot's drive train
    private DriveTrain driveTrain;

    private XboxController driveController;

    // Trajectory from csv
    private Trajectory traj;
    // Modifies center traj for tank setup
    private TankModifier modifier;
    // Wrappers for encoders
    private EncoderFollower lFollower, rFollower;

    private boolean allowDrive;

    // runs a thread with a certain timer interval
    public Notifier notifier;

    private CANEncoder rEnc, lEnc;

    public PathConverter(DriveTrain driveTrain, XboxController driveController, Trajectory traj) {
        allowDrive = true;
        this.driveTrain = driveTrain;
        this.traj = traj;

        rEnc = driveTrain.getEncoder('r');
        lEnc = driveTrain.getEncoder('l');

        // modify to our robot tank width
        modifier = new TankModifier(traj).modify(Constants.WHEELBASE_WIDTH);

        lFollower = new EncoderFollower();
        rFollower = new EncoderFollower();

        this.driveController = driveController;
    }

    /**
     * Sets up followers with default encoder and wheel info
     */
    public void setUpFollowers() {
        setUpFollowers(Constants.WHEEL_REV, Constants.WHEEL_DIAMETER);
    }

    /**
     * Sets up encoder followers
     * 
     * @param ticksInRev encoder ticks in a revolution
     * @param wheelDiam  diameter of the wheel in meters
     */
    public void setUpFollowers(int ticksInRev, double wheelDiam) {
        // pid values
        double p = 1, i = 0, d = 0;

        Robot.gyro.reset();
        lEnc.setPosition(0);
        rEnc.setPosition(0);

        // give the followers the trajectories (switch bexause it is buggy)
        lFollower.setTrajectory(modifier.getRightTrajectory());
        rFollower.setTrajectory(modifier.getLeftTrajectory());

        // give the wrappers our encoders, ticks, and wheel diam
        lFollower.configureEncoder(Math.abs((int) lEnc.getPosition()), ticksInRev, wheelDiam);
        rFollower.configureEncoder(Math.abs((int) rEnc.getPosition()), ticksInRev, wheelDiam);

        // configure pidva
        lFollower.configurePIDVA(p, i, d, 1 / Constants.MAX_VELOCITY, 0);
        rFollower.configurePIDVA(p, i, d, 1 / Constants.MAX_VELOCITY, 0);
    }

    /**
     * Runs the robot to follow the path
     */
    public void followPath() {
        allowDrive = false;
        // :: is lambda shortcut
        // pass is method to be used in thread
        notifier = new Notifier(this::followPathHelper);
        // give it the time interval, which is always 0.02
        notifier.startPeriodic(traj.get(0).dt);
    }

    /**
     * Helper method that is run through the notifier thread
     */
    private void followPathHelper() {
        // stop notifier when it is finished, or when movement detected from controllers
        if ((lFollower.isFinished() || rFollower.isFinished()) || checkControllers()) {
            // driveController.setRumble(RumbleType.kLeftRumble, 0.5);
            // driveController.setRumble(RumbleType.kRightRumble, 0.5);
            SmartDashboard.putBoolean("Pathfinder Job", true);
            allowDrive = true;
            notifier.stop();
        } else {
            // pause the pathfinder
            if (driveController.getBButtonPressed()) {
                long prevTime = System.currentTimeMillis();
                while ((System.currentTimeMillis() - prevTime) <= 4000) {
                    if (driveController.getBButtonPressed())
                        break;
                }
            }

            double l = 0, r = 0, gyro_heading = 0, desired_heading = 0, angleDifference = 0, turn = 0;
            double kG = 0;
            int constant = 1;

            // calculate the speed for the motors at this moment
            l = constant * lFollower.calculate(Math.abs((int) lEnc.getPosition()));
            r = constant * rFollower.calculate(Math.abs((int) rEnc.getPosition()));

            // calculate angle heading
            gyro_heading = -Robot.gyro.getYaw();
            desired_heading = -Pathfinder.r2d(lFollower.getHeading());
            angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
            kG = 0.8 * (-1.0 / 80.0);
            turn = kG * angleDifference;

            // run values through tankdrive
            driveTrain.getDrive().tankDrive((l + turn), (r - turn));

            workShuffleBoard();
        }

    }

    /**
     * Checks if the controller sticks have been moved
     * 
     * @return true if the controller has been touched, false if not
     */
    private boolean checkControllers() {
        return (driveController.getY(Hand.kLeft) > 0.3) || (driveController.getY(Hand.kLeft) < -0.3)
                || (driveController.getY(Hand.kRight) > 0.3) || (driveController.getY(Hand.kRight) < -0.3);
    }

    /**
     * @return the traj
     */
    public Trajectory getTrajectory() {
        return traj;
    }

    /**
     * @return the lFollower
     */
    public EncoderFollower getLeftFollower() {
        return lFollower;
    }

    /**
     * @return the rFollower
     */
    public EncoderFollower getRightFollower() {
        return rFollower;
    }

    /**
     * Method that informs other classes if notifier is done or not
     * 
     * @return true if notifier is finished, false if not
     */
    public boolean isDriveAllowed() {
        return allowDrive;
    }

    /**
     * Updates dashboard values
     */
    private void workShuffleBoard() {
        // SmartDashboard.putNumber("Right Drive Encoder Position", rEnc.getPosition());
        // SmartDashboard.putNumber("Right Drive Encoder Velocity (rpm)",
        // rEnc.getVelocity());

        // SmartDashboard.putNumber("Left Drive Encoder Position", lEnc.getPosition());
        // SmartDashboard.putNumber("Left Drive Encoder Velocity (rpm)",
        // lEnc.getVelocity());

        SmartDashboard.putNumber("Nav-X Angle", Robot.gyro.getAngle());
    }
}