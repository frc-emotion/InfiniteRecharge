package frc.robot.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Pivot {
    private XboxController operatorController;
    private CANSparkMax screwMotor;
    private DigitalInput lowerLimit, upperLimit;

    private double threshold, maxAngle, revToAngle, callibrateSpeed;
    private double teleopConstant;

    public Pivot(int motorPort, int maxCurrent, int lowerLimitPort, int upperLimitPort, double teleopConstant, double callibrateSpeed,
            double revToAngle, double threshold, double maxAngle, XboxController operatorController) {
        screwMotor = new CANSparkMax(motorPort, MotorType.kBrushless);

        screwMotor.setSecondaryCurrentLimit(maxCurrent);
        screwMotor.setSmartCurrentLimit(maxCurrent);
        screwMotor.setIdleMode(IdleMode.kBrake);

        lowerLimit = new DigitalInput(lowerLimitPort);
        upperLimit = new DigitalInput(upperLimitPort);

        this.operatorController = operatorController;
        this.teleopConstant = teleopConstant;
        this.threshold = threshold;
        this.callibrateSpeed = callibrateSpeed;
        this.maxAngle = maxAngle;
        this.revToAngle = revToAngle;
    }

    public void callibrate() {
        if (lowerLimit.get()) {
            screwMotor.set(0);
            screwMotor.getEncoder().setPosition(0);
        } else {
            screwMotor.set(callibrateSpeed);
        }
    }

    public double getAngle() {
        return screwMotor.getEncoder().getPosition() * revToAngle;
    }

    public void run() {
        if (Math.abs(operatorController.getY(Hand.kLeft)) > threshold) {
            setAngle(getAngle() + operatorController.getY(Hand.kLeft) * teleopConstant); // Currently exponential

        }
    }

    public void setAngle(double angle) {
        if (angle > maxAngle) {
            angle = maxAngle;
        } else if (angle < 0) {
            angle = 0;
        }

        if (lowerLimit.get() && angle <= screwMotor.getEncoder().getPosition() * revToAngle) {
            if (getAngle() != 0) {
                callibrate();
            }

            screwMotor.set(0);
        } else if (upperLimit.get() && angle >= screwMotor.getEncoder().getPosition() * revToAngle) {
            screwMotor.set(0);
        } else {
            screwMotor.getPIDController().setReference(angle / revToAngle, ControlType.kPosition);
        }
    }
}