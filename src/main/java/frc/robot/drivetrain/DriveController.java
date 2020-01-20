package frc.robot.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

/**
 * Class that handles the drive train controllers
 * 
 * <p>
 * Operation: Set the target RPM for each wheel with the respective set function
 * </p>
 * 
 * Note: Constants can be set after initialization
 * 
 * @author Ryan Chaiyakul
 */
class DriveController {
    private CANPIDController lPIDController, rPIDController; // SparkMAX PID controllers

    private boolean enabled; // Flag for sucessfully installation

    DriveController(CANEncoder lEncoder, CANEncoder rEncoder, CANPIDController lPIDController,
            CANPIDController rPIDController, double maxOutput, double kP) {
        this.lPIDController = lPIDController;
        this.rPIDController = rPIDController;

        lPIDController.setOutputRange(-maxOutput, maxOutput);
        lPIDController.setFeedbackDevice(lEncoder);

        rPIDController.setOutputRange(-maxOutput, maxOutput);
        rPIDController.setFeedbackDevice(rEncoder);

        setKP(kP);

        enabled = true;
    }

    DriveController(CANEncoder lEncoder, CANEncoder rEncoder, CANPIDController lPIDController,
            CANPIDController rPIDController, double maxOutput, double kP, double kI) {
        this(lEncoder, rEncoder, lPIDController, rPIDController, maxOutput, kP);

        setKI(kI);
    }

    DriveController(CANEncoder lEncoder, CANEncoder rEncoder, CANPIDController lPIDController,
            CANPIDController rPIDController, double maxOutput, double kP, double kI, double kD) {
        this(lEncoder, rEncoder, lPIDController, rPIDController, maxOutput, kP, kI);

        setKD(kD);
    }

    /**
     * Set kP for both controllers
     * 
     * @param kP
     */
    public void setKP(double kP) {
        lPIDController.setP(kP);
        rPIDController.setP(kP);
    }

    /**
     * Set kI for both controllers
     * 
     * @param kI
     */
    public void setKI(double kI) {
        lPIDController.setI(kI);
        rPIDController.setI(kI);
    }

    /**
     * Set kD for both controllers
     * 
     * @param kD
     */
    public void setKD(double kD) {
        lPIDController.setD(kD);
        rPIDController.setD(kD);
    }

    /**
     * Set left velocity refrence
     * 
     * @param velocity
     */
    public void setLVelocity(double velocity) {
        if (!enabled) {
            return;
        }

        lPIDController.setReference(velocity, ControlType.kVelocity);
    }

    /**
     * Set right velocity refrence
     * 
     * @param velocity
     */
    public void setRVelocity(double velocity) {
        if (!enabled) {
            return;
        }

        rPIDController.setReference(velocity, ControlType.kVelocity);
    }

    /**
     * Returns enabled flag
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Disables controllers
     */
    public void disable() {
        enabled = false;
    }

    /**
     * Enables controllers
     */
    public void enable() {
        enabled = true;
    }
}