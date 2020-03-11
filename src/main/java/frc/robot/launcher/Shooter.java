package frc.robot.launcher;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Class that handles the shooting mechanism
 * 
 * <p>
 * Usage: Call run() during teleop.
 * </p>
 * 
 * <p>
 * Autonomous: Call functions every cycle of autonomousPeriodic
 * </p>
 * 
 * @author Ryan Chaiyakul
 */
public class Shooter {
    private CANSparkMax sparkA, sparkB; // (L, R)
    private DoubleSolenoid shooterSolenoid; // (closed, open)
    private double startTime; // stores start time of macro call

    public Shooter() {
        // workShuffleBoard();
        // Intialize motors
        sparkA = new CANSparkMax(Constants.SHOOTER_PORTS[0], MotorType.kBrushless);
        sparkB = new CANSparkMax(Constants.SHOOTER_PORTS[1], MotorType.kBrushless);

        ArrayList<CANSparkMax> shooterSparkMax = new ArrayList<CANSparkMax>() {
            {
                add(sparkA);
                add(sparkB);
            }
        };

        for (CANSparkMax spark : shooterSparkMax) {
            spark.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
            spark.setIdleMode(IdleMode.kCoast); // Coast mode in order to prevent breaking between shots
        }

        shooterSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_SHOOTER_PORT[0], Constants.PNEUMATIC_SHOOTER_PORT[1]);

        startTime = 0;
        sparkA.getPIDController().setP(Constants.SHOOTER_KP);
        sparkA.getPIDController().setI(Constants.SHOOTER_KI);
        sparkA.getPIDController().setD(Constants.SHOOTER_KD);
        sparkA.getPIDController().setFF(Constants.SHOOTER_KF);
    }

    public double getRPM() {
        return sparkA.getEncoder().getVelocity();
    }

    /**
     * Call periodically in teleopPeriodic
     */
    public void run() {
        if (Robot.operatorController.getTriggerAxis(Hand.kRight) >= Constants.TRIGGER_THRESHOLD) {
            shoot();
        } else if (Robot.operatorController.getStartButton()) {
            turn();
        } else if (Robot.operatorController.getBackButton()) {
            rotate();
        } else {
            stop();
        }
    }

    /**
     * Handles both motors and pnuematic timing
     */
    public void shoot() {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
        }
        spinUp();

        // Wait for .7 of a second before opening hook. Might change to activate based
        // on current RPM
        if (System.currentTimeMillis() - startTime > Constants.SHOOTER_SHOOT_TIME) {
            open();
        }
    }

    /**
     * Opens tube for shooting
     */
    public void open() {
        shooterSolenoid.set(Value.kReverse);
    }

    /**
     * Closes tube for shooting
     */
    public void close() {
        shooterSolenoid.set(Value.kForward);
    }

    /**
     * Uses PID to maintain the shooter rpm at 5100 rpm
     */
    public void spinUp() {
        sparkA.getPIDController().setReference(Constants.SHOOTER_TARGET_RPM, ControlType.kVelocity);
        sparkB.set(-sparkA.getAppliedOutput());

    }

    /**
     * Stop motors
     */
    public void spinDown() {
        sparkA.getPIDController().setReference(0, ControlType.kDutyCycle);
        sparkA.set(0);
        sparkB.set(0);
    }

    /**
     * Turn motors slowly in the same direction
     */
    public void turn() {
        sparkA.set(Constants.SHOOTER_TURN_SPEED);
        sparkB.set(Constants.SHOOTER_TURN_SPEED);
    }

    /**
     * Calls turn until the wheel completes 2-3 rotations
     */
    public void rotate() {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - startTime > Constants.SHOOTER_ROTATE_TIME) {
            spinDown();
            return;
        }
        turn();
    }

    /**
     * sets motors and pneumatics to default state and resets macros
     */
    public void stop() {
        close();
        spinDown();
        startTime = 0; // Resets macros timing
    }
}