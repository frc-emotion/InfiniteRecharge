package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ? Add feedbackPIDController if necessary
// TODO Figure out what allows the balls to enter the shooting chamber and create a shoot function
/**
 * Class that runs the Shooting Mechanism
 * 
 * <p>
 * Autonomous: Four public functions should be utilized. The controller for
 * speed control should be set in robotInit.
 * <ul>
 * <li>spinUp: Using the set control method, the two Neo motors will begin to
 * accelerate the wheel until the set targetRPM is reached. Call in
 * autonomousPeriodic as this function only sets the speed once then exits.</li>
 * <li>spinDown: Sets the idleMode to IdleMode.kBrake and sets the motor speed
 * to off. Only needs to be called once.</li>
 * <li>update: Resets variables every autonomous cycle before running spinUp or
 * spinDown.</li>
 * <li>dashboardRun: Update SmartDashboard with the new variables</li>
 * 
 * </ul>
 * </p>
 * <p>
 * Operation: 'A' button on the operator controller will activate the shooting
 * mechanism. Will stop when released.
 * </p>
 * <p>
 * Note: All constants are passed through constructors or set functions except
 * for what button to press for activation. Do not call update or dashboardRun
 * in teleopPeriodic as it is called by run.
 * </p>
 * 
 * @author Ryan Chaiyakul
 */
class Shooter {
    private XboxController shooterController; // Controller checked during run()
    private CANSparkMax sparkA, sparkB; // Two Neos are utilized
    private SpeedControllerGroup shooterGroup; // Groups sparkA and sparkB to the same set function
    private CANEncoder sparkEncoder; // sparkA's builtin encoder

    private double targetRPM, currentRPM, threshold; // Generic constants
    private boolean twoStepController, pidController; // Booleans that determine what control function will be called

    private double minOutput, maxOutput; // Lower limit and upper limit of the 2 Step Controller

    private float kP, kI, kD; // Constants for PID Controller
    private PIDControl shooterPIDControl; // PID Controller

    private double output; // Variable for spinUp
    private boolean inRange; // Flag for whether the the targetRPM is reached

    private boolean activated; // Flag ofr whether spinUp was triggered this cycle

    public Shooter(int[] ports, int maxCurrent, double targetRPM, double threshold, double maxOutput,
            XboxController shooterController) {
        // Ports should be length 2
        if (ports.length != 2) {
            return;
        }

        sparkA = new CANSparkMax(ports[0], MotorType.kBrushless);
        sparkB = new CANSparkMax(ports[1], MotorType.kBrushless);

        // Group sparkA and sparkB into an ArrayList to set both motors with a for loop
        ArrayList<CANSparkMax> shooterSparkMax = new ArrayList<CANSparkMax>() {
            {
                add(sparkA);
                add(sparkB);
            }
        };

        for (CANSparkMax spark : shooterSparkMax) {
            spark.setSmartCurrentLimit(maxCurrent);
            spark.setSecondaryCurrentLimit(maxCurrent);
            spark.setIdleMode(IdleMode.kBrake); // IdleMode will be changed dynamically
        }

        sparkEncoder = sparkA.getEncoder();
        shooterGroup = new SpeedControllerGroup(sparkA, sparkB);

        // Store variables
        this.shooterController = shooterController;
        this.targetRPM = targetRPM;
        this.threshold = threshold;
        this.maxOutput = maxOutput;

        // Set default values

        twoStepController = pidController = false;
        currentRPM = 0;
        minOutput = 0;
        kP = kI = kD = 0;
    }

    /**
     * Returns whether the target speed is acquired
     * 
     * @return boolean if in range
     */
    public boolean isInRange() {
        return inRange;
    }

    /**
     * Sets both motors to IdleMode.kCoast
     */
    private void setCoastMode() {
        if (sparkA.getIdleMode() != IdleMode.kCoast) {
            sparkA.setIdleMode(IdleMode.kCoast);
            sparkB.setIdleMode(IdleMode.kCoast);
        }
    }

    /**
     * Sets both motors to IdleMode.kBrake
     */
    private void setBrakeMode() {
        if (sparkA.getIdleMode() != IdleMode.kBrake) {
            sparkA.setIdleMode(IdleMode.kBrake);
            sparkB.setIdleMode(IdleMode.kBrake);
        }
    }

    /**
     * Enables Two Step Controller using the default lower limit of 0
     */
    public void enableTwoStepController() {
        twoStepController = true;
    }

    /**
     * Enables Two Step Controller using the passed lower limit
     * 
     * @param minOutput lower limit of Two Step Controller
     */
    public void enableTwoStepController(double minOutput) {
        this.minOutput = minOutput;
        twoStepController = true;
    }

    /**
     * Disables Two Step Controller but does not reset set values.
     */
    public void disableTwoStepController() {
        twoStepController = false;
    }

    /**
     * Enables PID Controller with P only
     * 
     * @param kP constant for P
     */
    public void enablePIDController(float kP) {
        this.kP = kP;
        shooterPIDControl = new PIDControl(kP, kI, kD);
        shooterPIDControl.setMaxSpeed(maxOutput);
        shooterPIDControl.setTolerance(threshold);

    }

    /**
     * Enables PID Controller with P and I
     * 
     * @param kP constant for P
     * @param kI constant for I
     */
    public void enablePIDController(float kP, float kI) {
        this.kI = kI;
        enablePIDController(kP);
    }

    /**
     * Enables PID Controller with P, I, and D
     * 
     * @param kP constant for P
     * @param kI constant for I
     * @param kD constant for D
     */
    public void enablePIDController(float kP, float kI, float kD) {
        this.kD = kD;
        enablePIDController(kP, kI);
    }

    /**
     * Disables PID Controller but does not reset constants.
     */
    public void disablePIDController() {
        pidController = false;
    }

    /**
     * Function that should be called by teleopPeriodic
     */
    public void run() {
        update();
        // A Button should be held to activate
        if (shooterController.getAButton()) {
            spinUp(); // spinUp should be self contained as it can be called during autonomous
            activated = true;
        } else {
            spinDown();
        }
        dashboardRun();
    }

    /**
     * Function that should be called periodically for autonomous. Only exposed for
     * autonomous in order to insure continually updates to SmartDashboard.
     */
    public void update() {
        currentRPM = sparkEncoder.getVelocity() * sparkEncoder.getVelocityConversionFactor();
        output = 0;
        inRange = false;
        activated = false;
    }

    /**
     * Function that should be called before autonomousPeriodic ends in autonomous.
     * Only exposed for autonomous in order to insure continually updates to
     * SmartDashboard.
     */
    public void dashboardRun() {
        SmartDashboard.putNumber("ShooterAngularVelocity", currentRPM * 6);
        SmartDashboard.putNumber("ShooterOutput", output);
        SmartDashboard.putBoolean("ShooterVelocityInRange", inRange);
        SmartDashboard.putBoolean("ShooterActivated", activated);
    }

    /**
     * Function called by run or autonomousPeriodic periodically to set motor speeds
     */
    public void spinUp() {
        setCoastMode();
        if (pidController) {
            output = runPIDController();
            inRange = shooterPIDControl.isInRange();
        } else if (twoStepController) {
            output = runTwoStepController();
            if (output == minOutput) {
                inRange = true;
            }
        }

        shooterGroup.set(output);
    }

    /**
     * Function called by run or autonomous periodically to set motors to off
     */
    public void spinDown() {
        shooterGroup.set(0);
        setBrakeMode();
    }

    /**
     * Function called by spinUp to get output value from PID Controller
     * 
     * @return output double value that shooterGroup will be set to
     */
    private double runPIDController() {
        return shooterPIDControl.getValue(targetRPM, currentRPM);
    }

    /**
     * Function called by spinUp to get output value from Two Step Controller
     * 
     * @return output double value that shooterGroup will be set to
     */
    private double runTwoStepController() {
        // Might remove if not necessary. Just incase if the currentRPM exceeds
        // targetRPM before the motor is stopped.
        if (targetRPM - currentRPM < 0) {
            return minOutput;
        }
        if (Math.abs(targetRPM - currentRPM) < threshold) {
            return minOutput;
        }
        return maxOutput;
    }
}