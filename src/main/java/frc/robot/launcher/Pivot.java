package frc.robot.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PIDControl;
import frc.robot.Robot;

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
<<<<<<< HEAD
    private PIDControl pidControl;
=======
    private boolean usingMacro = false;
>>>>>>> f324c34fd8d83a147a94915d97f2d4b83176d1ae

    public Pivot() {
        sparkA = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        lowerLimit = new DigitalInput(Constants.PIVOT_LIMIT_PORT);
        alignment = new Alignment();
<<<<<<< HEAD
        pidControl = new PIDControl(Constants.PIVOT_KP, Constants.PIVOT_KI, Constants.PIVOT_KD);
        pidControl.setTolerance(Constants.PIVOT_THRESHOLD);
        pidControl.setMaxSpeed(Constants.PIVOT_AUTO_SPEED);
=======

        workShuffleBoard();
>>>>>>> f324c34fd8d83a147a94915d97f2d4b83176d1ae
    }

    public void callibrate() {
        sparkA.set(0);
        sparkA.getEncoder().setPosition(0);
    }

    /**
     * Converted revolutions to a positive number
     * 
     * @return
     */
    public double getRevolution() {
        return -sparkA.getEncoder().getPosition();
    }

    public boolean atRev(double angle) {
        return Math.abs(angle - getRevolution()) < Constants.PIVOT_THRESHOLD;
    }

    public void run() {
        workShuffleBoard();
        System.out.println(getRevolution());
        if (Math.abs(Robot.operatorController.getY(Hand.kLeft)) > Constants.TRIGGER_THRESHOLD) {
            teleopRun();
        } else if (Robot.operatorController.getBButton()) {
            // align();
        } else if (Robot.operatorController.getXButton()) {
            callibrate();
        } else {
            // DPad controls
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    usingMacro = true;
                    setAgainst();
                    break;
                case 90:
                    // Right
                    usingMacro = true;
                    setWheel();
                    break;
                case 180:
                    // Down
                    usingMacro = true;
                    setTrench();
                    break;
                case 270:
                    // Left
                    usingMacro = true;
                    setLine();
                    break;
                default:
                    // Catches -1 or status when nothing is pressed on DPad
                    stop();
                    
            }
        }
    }

    public void stop() {
        usingMacro = false;
        sparkA.set(0);
        pidControl.cleanup();
    }

    public void teleopRun() {
        // Prevent pivot to go below the lowerLimit switch
        if (!lowerLimit.get() && Robot.operatorController.getY(Hand.kLeft) > 0) {
            callibrate();
            return;
        }

        if (getRevolution() < Constants.PIVOT_ZERO_THRESHOLD && Robot.operatorController.getY(Hand.kLeft) > 0) {
            sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_ZERO_SPEED);
            return;
        }

        if (getRevolution() > Constants.PIVOT_MAX_REVOLUTION && Robot.operatorController.getY(Hand.kLeft) < 0) {
            return;
        }
        sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_TELEOP_SPEED);
    }

    public void align() {
        setRevolution(alignment.getAngle() * Constants.RADIANS_TO_REV);
    }

    public void setAgainst() {
        setRevolution(79);
    }

    public void setLine() {
        setRevolution(23.78);
    }

    public void setWheel() {
        setRevolution(33);
    }

    public void setTrench() {
        setRevolution(13.7);
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

    public void setRevolution(double rev) {
        double speed = -pidControl.getValue(rev, getRevolution());

        // Prevent pivot to go below the lowerLimit switch
        if (!lowerLimit.get() && speed > 0) {
            callibrate();
            return;
        }

        sparkA.set(speed);
    }

    public void workShuffleBoard() {
        SmartDashboard.putNumber("Current Revolutions", getRevolution());

    }
}