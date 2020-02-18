package frc.robot.launcher;

import frc.robot.limelight.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivot {
    private XboxController operatorController;
    private CANSparkMax screwMotor;
    private DigitalInput lowerLimit;

    private double controllerThreshold, angleThreshold, maxAngle, revToAngle, refrenceAngle, callibrateSpeed;
    private double mountingAngle;
    private double teleopConstant;

    private Alignment alignment;

    public Pivot(int motorPort, int maxCurrent, int lowerLimitPort, double teleopConstant, double callibrateSpeed,
            double revToAngle, double controllerThreshold, double angleThreshold, double maxAngle,
            XboxController operatorController) {
        screwMotor = new CANSparkMax(motorPort, MotorType.kBrushless);

        screwMotor.setSecondaryCurrentLimit(maxCurrent);
        screwMotor.setSmartCurrentLimit(maxCurrent);
        screwMotor.setIdleMode(IdleMode.kBrake);

        lowerLimit = new DigitalInput(lowerLimitPort);

        this.operatorController = operatorController;
        this.teleopConstant = teleopConstant;
        this.controllerThreshold = controllerThreshold;
        this.angleThreshold = angleThreshold;
        this.callibrateSpeed = callibrateSpeed;
        this.maxAngle = maxAngle;
        this.revToAngle = revToAngle;

        refrenceAngle = getAngle();
    }

    public void enablePID(double kP, double kI, double kD) {
        screwMotor.getPIDController().setP(kP);
        screwMotor.getPIDController().setI(kI);
        screwMotor.getPIDController().setD(kD);

        screwMotor.getPIDController().setOutputRange(-0.2, 0.2);
    }

    public void enablePID(double kP, double kI) {
        enablePID(kP, kI, 0);
    }

    public void enablePID(double kP) {
        enablePID(kP, 0);
    }

    public void enableAlignment(double mountingHeight, double mountingAngle, double refrenceHeight, int pipeline,
            double maxVelocity) {
        this.alignment = new Alignment(new Distance(mountingHeight, mountingAngle, refrenceHeight), maxVelocity,
                pipeline);

        this.mountingAngle = mountingAngle;
    }

    public void callibrate() {
        if (!lowerLimit.get()) {
            screwMotor.set(0);
            screwMotor.getEncoder().setPosition(0);
            refrenceAngle = getAngle();
        } else {
            screwMotor.set(callibrateSpeed);
        }
    }

    public double getAngle() {
        return screwMotor.getEncoder().getPosition() * revToAngle;
    }

    public boolean isInRange() {
        return Math.abs(getAngle() - refrenceAngle) < angleThreshold;
    }

    public void run() {
        if (Math.abs(operatorController.getY(Hand.kLeft)) > controllerThreshold) {
            screwMotor.set(operatorController.getY(Hand.kLeft) * teleopConstant);
        } else {
            screwMotor.set(0);
        }

        SmartDashboard.putNumber("refrenceAngle", refrenceAngle);
        SmartDashboard.putNumber("currentRevolution", screwMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("currentAngle", getAngle());
    }

    public void align() {
        if (alignment.getAngle() == -1) {
            return;
        }
        setAngle(alignment.getAngle() - mountingAngle);
    }

    public void setAngle(double angle) {
        if (angle > maxAngle) {
            angle = maxAngle;
        } else if (angle < 0) {
            angle = 0;
        }

        if (!lowerLimit.get() && angle <= screwMotor.getEncoder().getPosition() * revToAngle) {
            if (getAngle() != 0) {
                callibrate();
            }

            screwMotor.set(0);
        } else {
            screwMotor.getPIDController().setReference(angle / revToAngle, ControlType.kPosition);
        }
    }

    public void setAngle() {
        setAngle(refrenceAngle);
    }
}