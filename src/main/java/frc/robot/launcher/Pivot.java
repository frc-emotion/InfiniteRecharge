package frc.robot.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
    private DigitalOutput lowerLimit; // lowerLimit is active low

    public Pivot() {
        sparkA = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
        lowerLimit = new DigitalOutput(Constants.PIVOT_LIMIT_PORT);
    }

    public void callibrate() {
        if (!lowerLimit.get()) {
            sparkA.set(0);
            sparkA.getEncoder().setPosition(0);
        }
    }

    /**
     * Converted revolutions to a positive number
     * 
     * @return
     */
    public double getRevolution() {
        return -sparkA.getEncoder().getPosition();
    }

    public void run() {
        // DEBUGGING
        SmartDashboard.putNumber("PivotRevolution", getRevolution());
        SmartDashboard.putBoolean("AtLowerLimit", !lowerLimit.get());

        if (Math.abs(Robot.operatorController.getY(Hand.kLeft)) > Constants.TRIGGER_THRESHOLD) {
            teleopRun();
        } else if (Robot.operatorController.getBButton()) {
            align();
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
                case 180:
                    // Down
                    setTrench();
                case 270:
                    // Left
                    setLine();
                    break;
                default:
                    // Catches -1 or status when nothing is pressed on DPad
                    sparkA.set(0);
            }
        }
    }

    public void teleopRun() {
        // Prevent pivot to go below the lowerLimit switch
        if (!lowerLimit.get() && Robot.operatorController.getY(Hand.kLeft) > 0) {
            callibrate();
            return;
        }

        sparkA.set(Robot.operatorController.getY(Hand.kLeft) * Constants.PIVOT_TELEOP_SPEED);
    }

    public void align() {
    }

    public void setAgainst() {
        setRevolution(0);
    }

    public void setLine() {
        setRevolution(0);
    }

    public void setWheel() {
        setRevolution(0);
    }

    public void setTrench() {
        setRevolution(0);
    }

    public void setRevolution(double rev) {
        int sign = -1;
        if (rev < getRevolution()) {
            sign *= -1;
        }

        // Prevent pivot to go below the lowerLimit switch
        if (!lowerLimit.get() && sign > 0) {
            callibrate();
            return;
        }

        if (Math.abs(getRevolution() - rev) < Constants.PIVOT_THRESHOLD) {
            sparkA.set(0);
            return;
        }
        sparkA.set(sign * Constants.PIVOT_AUTO_SPEED);
    }
}