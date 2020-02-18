package frc.robot.launcher;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
public class Shooter {
    private XboxController shooterController; // Controller checked during run()
    private CANSparkMax sparkA, sparkB; // Two Neos are utilized
    private CANEncoder sparkEncoder; // sparkA's builtin encoder

    private DoubleSolenoid hook; // Holds the balls back until ready to shoot

    private double targetRPM, currentRPM, thresholdRPM, thresholdTrigger; // Generic controller constants
    private boolean twoStepController, pidController; // Booleans that determine what control function will be called

    private double minOutput, maxOutput; // Lower limit and upper limit of the 2 Step Controller

    private float kI, kD; // Constants for PID Controller

    private double output; // Variable for spinUp

    private boolean activated; // Flag for whether spinUp was triggered this cycle

    private Pivot pivot;

    public Shooter(int[] ports, int forwardChannel, int reverseChannel, int maxCurrent, double targetRPM,
            double waitTime, double thresholdRPM, double thresholdTrigger, double maxOutput,
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
            spark.setIdleMode(IdleMode.kCoast);
        }

        sparkEncoder = sparkA.getEncoder();

        hook = new DoubleSolenoid(forwardChannel, reverseChannel);

        // Store variables
        this.shooterController = shooterController;
        this.targetRPM = targetRPM;
        this.thresholdRPM = thresholdRPM;
        this.thresholdTrigger = thresholdTrigger;
        this.maxOutput = maxOutput;

        // Set default values

        twoStepController = pidController = false;
        currentRPM = 0;
        minOutput = 0;
        kI = kD = 0;
    }

    public void enablePivot(int motorPort, int maxCurrent, int lowerLimitPort, double teleopConstant,
            double callibrateSpeed, double revToAngle, double controllerThreshold, double angleThreshold,
            double maxAngle, double mountingHeight, double mountingAngle, double refrenceHeight, int pipeline,
            double maxVelocity) {
        pivot = new Pivot(motorPort, maxCurrent, lowerLimitPort, teleopConstant, callibrateSpeed, revToAngle,
                controllerThreshold, angleThreshold, maxAngle, shooterController);
        pivot.enableAlignment(mountingHeight, mountingAngle, refrenceHeight, pipeline, maxVelocity);

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
        CANPIDController sparkPIDController = sparkA.getPIDController();

        sparkPIDController.setP(kP);
        sparkPIDController.setI(kI);
        sparkPIDController.setD(kD);

        sparkPIDController.setOutputRange(-maxOutput, maxOutput);
        sparkPIDController.setFeedbackDevice(sparkEncoder);
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

    private boolean atTargetRPM() {
        return Math.abs(targetRPM - currentRPM) < thresholdRPM;
    }

    /**
     * Function that should be called by teleopPeriodic
     */
    public void run() {
        update();

        // Trigger Button should be held to activate
        if (shooterController.getTriggerAxis(Hand.kRight) >= thresholdTrigger) {
            shoot();
        } else {
            // pivot.run();
            spinDown();
            hook.set(Value.kForward);
        }

        dashboardRun();
    }

    /**
     * Function that should be called periodically for autonomous. Only exposed for
     * autonomous in order to insure continually updates to SmartDashboard.
     */
    public void update() {
        currentRPM = sparkEncoder.getVelocity();
        output = 0;
        activated = false;
    }

    /**
     * Function that should be called before autonomousPeriodic ends in autonomous.
     * Only exposed for autonomous in order to insure continually updates to
     * SmartDashboard.
     */
    public void dashboardRun() {
        SmartDashboard.putNumber("ShooterRPM", currentRPM);
        SmartDashboard.putNumber("ShooterOutput", output);
        SmartDashboard.putBoolean("ShooterVelocityInRange", atTargetRPM());
        SmartDashboard.putBoolean("ShooterActivated", activated);
    }

    /**
     * Calls spinUp and releases the ball once the targetRPM is reached
     */
    public void shoot() {
        spinUp();
        // pivot.setAngle();

        hook.set(Value.kReverse);
    }

    /**
     * Function called by run or autonomousPeriodic periodically to set motor speeds
     */
    public void spinUp() {
        if (pidController) {
            output = runPIDController();
        } else if (twoStepController) {
            output = runTwoStepController();
        }

        sparkA.set(output);
        sparkB.set(-output);
    }

    /**
     * Function called by run or autonomous periodically to set motors to off
     */
    public void spinDown() {
        sparkA.set(0);
        sparkB.set(0);
    }

    /**
     * Function called by spinUp to get output value from PID Controller
     * 
     * @return output double value that shooterGroup will be set to
     */
    private double runPIDController() {
        sparkA.getPIDController().setReference(targetRPM, ControlType.kVelocity);
        return sparkA.get();
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
        if (atTargetRPM()) {
            return minOutput;
        }
        return maxOutput;
    }
}