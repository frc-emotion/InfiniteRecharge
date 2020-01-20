package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

/**
 * Class that generates the trajectory and provides wheel speeds to maintain
 * course
 * 
 * <p>
 * Operation: Call followTrajectory periodically which provides new wheelSpeeds
 * to maintain.
 * </p>
 * 
 * Note: followTrajectory returns null if the object is not configured properly.
 * The trajectory is in progress until the trajectory is reset.
 * 
 * @author Ryan Chaiykaul
 */
public class FollowTrajectory {
    private DifferentialDriveKinematics driveKinematics; // Kinematics object
    private RamseteController trajectoryController; // RamseteController object
    private boolean ramseteController; // Flag for Ramsete Controller

    private Trajectory trajectory; // Current trajectory to follow
    private long startingTime; // Time followTrajectory was first called for this trajectory
    private boolean inProgress; // Flag whether a followTrajectory is in progress

    FollowTrajectory(DifferentialDriveKinematics driveKinematics) {
        this.driveKinematics = driveKinematics;

        ramseteController = false;
        resetTrajectory();
    }

    FollowTrajectory(DifferentialDriveKinematics driveKinematics, double b, double zeta) {
        this(driveKinematics);

        enableRamseteController(b, zeta);
    }

    /**
     * Returns whether the trajectory reached the end Pose. Defaults to false if
     * trajectory is not in progress.
     * 
     * @return
     */
    public boolean isInRange() {
        if (inProgress) {
            return trajectoryController.atReference();
        }
        return false;
    }

    /**
     * Returns the inProgress flag
     */
    public boolean isInProgress() {
        return inProgress;
    }

    /**
     * Returns the time of the current trajectory in seconds
     * 
     * @return
     */
    public double getTime() {
        if (trajectory.equals(null)) {
            return 0;
        }

        return trajectory.getTotalTimeSeconds();
    }

    /**
     * Enables Ramsete controller with the passed constants
     * 
     * @param b    b constant for Ramsete
     * @param zeta zeta constant for Ramsete
     */
    public void enableRamseteController(double b, double zeta) {
        trajectoryController = new RamseteController(b, zeta);
        ramseteController = true;
    }

    /**
     * Disables Ramsete controller and disables current trajectory
     */
    public void disableRamseteController() {
        ramseteController = false;
        resetTrajectory();
    }

    /**
     * Updates the trajectory
     */
    public void setTrajectory(Trajectory trajectory) {
        resetTrajectory();
        this.trajectory = trajectory;
    }

    /**
     * Removes current trajectory and removes inProgress flag
     */
    public void resetTrajectory() {
        inProgress = false;
        trajectory = null;
    }

    /**
     * Returns the current wheelSpeed to maintain course
     * 
     * @param currentPose pose from odometry
     * 
     * @return
     */
    public DifferentialDriveWheelSpeeds followTrajectory(Pose2d currentPose) {
        if (ramseteController || trajectory.equals(null)) {
            return null;
        }

        if (!inProgress) {
            inProgress = true;
            startingTime = System.currentTimeMillis();
        }
        long currentTime = (System.currentTimeMillis() - startingTime) * 1000;

        Trajectory.State targetPose = trajectory.sample(currentTime);
        ChassisSpeeds adjustedSpeeds = trajectoryController.calculate(currentPose, targetPose);
        return driveKinematics.toWheelSpeeds(adjustedSpeeds);
    }

    /**
     * Call periodically to update variables and SmartDashboard
     */
    public void run() {
        double trajectoryTime = 0;
        if (!trajectory.equals(null)) {
            trajectoryTime = trajectory.getTotalTimeSeconds();
        }

        SmartDashboard.putNumber("TrajectoryTime", trajectoryTime);
        SmartDashboard.putBoolean("TrajectoryInProgress", inProgress);
    }
}