package frc.robot.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants;
import frc.robot.PIDControl;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Class that handles the pivot/screw mechanism that rotates the launcher
 * mechanism
 * 
 * <p>
 * Note: (- = upwards, + = downwards)
 * 
 * @author Ryan Chaiyakul
 */
public class Pivot {
    
    private CANSparkMax sparkA; // Spark to control pivot location with screw mechanism
    private Alignment alignment; // Alignment object
    private DigitalInput lowerLimit; // lowerLimit is active low
    private PIDControl pidControl;

    public Pivot() {
        sparkA = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        lowerLimit = new DigitalInput(Constants.PIVOT_LIMIT_PORT);
        alignment = new Alignment();

        pidControl = new PIDControl(Constants.PIVOT_KP, Constants.PIVOT_KI, Constants.PIVOT_KD);
        pidControl.setTolerance(Constants.PIVOT_THRESHOLD);
        pidControl.setMaxSpeed(Constants.PIVOT_AUTO_SPEED);

        sparkA.getEncoder().setPosition(0);
        initShuffleBoard();
    }

    /**
     * Converted revolutions to a positive number
     * 
     * @return
     */
    public double getRevolution() {
        return -sparkA.getEncoder().getPosition();
    }

    /**
     * Returns true if the current revolution is roughly the same as the provided
     * revolution
     * 
     * <p>
     * Note: has a threshold to determine if in range
     * 
     * @param revolution
     * @return
     */
    public boolean atRev(double revolution) {
        return Math.abs(revolution - getRevolution()) < Constants.PIVOT_THRESHOLD;
    }

    public void run() {
        workShuffleBoard();
        if (Math.abs(Robot.operatorController.getY(Hand.kLeft)) > Constants.TRIGGER_THRESHOLD) {
            teleopRun();
        } else if (Robot.operatorController.getBButton()) {
            // align();
        } else {
            // DPad controls
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    setAgainst();
                    break;
                case 90:
                    // Right
                    setWheel();
                    break;
                case 180:
                    // Down
                    setTrench();
                    break;
                case 270:
                    // Left
                    setLine();
                    break;
                default:
                    // Catches -1 or status when nothing is pressed on DPad
                    stop();

            }
        }
    }

    /**
     * Stop motor and reset macros
     */
    public void stop() {
        sparkA.set(0);
        pidControl.cleanup();
    }

    /**
     * Converts joystick controls to pivot speed
     */
    public void teleopRun() {
        // Prevent pivot from going below the lowerLimit switch
        if (!lowerLimit.get() && Robot.operatorController.getY(Hand.kLeft) > 0) {
            stop();
            return;
        }

        // Slows down pivot speed as it aproaches the lowest position
        if (getRevolution() < Constants.PIVOT_ZERO_THRESHOLD && Robot.operatorController.getY(Hand.kLeft) > 0) {
            sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_ZERO_SPEED);
            return;
        }

        // Prevents pivot from going above the max position
        if (getRevolution() > Constants.PIVOT_MAX_REVOLUTION && Robot.operatorController.getY(Hand.kLeft) < 0) {
            return;
        }

        sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_TELEOP_SPEED);
    }

    public void align() {
        setRevolution(alignment.getAngle() * Constants.RADIANS_TO_REV);
    }

    public void setAgainst() {
        setRevolution(78);
    }

    public void setLine() {
        setRevolution(18.5);
    }

    public void setWheel() {
        setRevolution(33);
    }

    public void setTrench() {
        setRevolution(14.25);
    }

    public void setBottom() {
        setRevolution(0);
    }

    public boolean atAlign() {
        return atRev(alignment.getAngle() * Constants.RADIANS_TO_REV);
    }

    public boolean atAgainst() {
        return atRev(79);
    }

    public boolean atLine() {
        return atRev(24.78);
    }

    public boolean atWheel() {
        return atRev(33);
    }

    public boolean atTrench() {
        return atRev(12.13);
    }

    public boolean atBottom() {
        return atRev(0);
    }

    /**
     * Uses PID to go to the desired revolution
     */
    public void setRevolution(double rev) {
        double speed = Constants.PIVOT_AUTO_SPEED;
        int sign = -1;
        if (rev < getRevolution()) {
            sign *= -1;
            speed -= 0.05;
        }

        // Prevent pivot to go below the lowerLimit switch
        if (!lowerLimit.get() && sign > 0 || getRevolution() > Constants.PIVOT_MAX_REVOLUTION) {
            stop();
            return;
        }

        if (Math.abs(getRevolution() - rev) < Constants.PIVOT_THRESHOLD) {
            stop();
            return;
        }
        sparkA.set(sign * speed);

        /**
         * double speed = -pidControl.getValue(rev, getRevolution());
         * 
         * // Prevent pivot to go below the lowerLimit switch if (!lowerLimit.get() &&
         * speed > 0) { stop(); return; } if (getRevolution() >
         * Constants.PIVOT_MAX_REVOLUTION) { stop(); return; }
         * 
         * sparkA.set(speed);
         */
    }

    public void initShuffleBoard() {
        SmartDashboard.putNumber("Pivot Position", getRevolution());
    }

    public void workShuffleBoard(){
        SmartDashboard.putNumber("Pivot Position", getRevolution());
    }
    
}