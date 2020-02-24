package frc.robot.launcher;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Intake {
    private XboxController operatorController;
    private CANSparkMax sparkA, sparkB, sparkC, sparkD;

    private DoubleSolenoid intakeSolenoid;

    private double tubeOutput;
    private double intakeOutput;
    private double threshold;

    private boolean intake;

    public Intake(int[] ports, int maxCurrent, int forwardPort, int reversePort, double intakeOutput, double tubeOutput,
            double threshold, XboxController operatorController) {
        if (ports.length != 4) {
            return;
        }

        sparkA = new CANSparkMax(ports[0], MotorType.kBrushless);
        sparkB = new CANSparkMax(ports[1], MotorType.kBrushless);
        sparkC = new CANSparkMax(ports[2], MotorType.kBrushless);
        sparkD = new CANSparkMax(ports[3], MotorType.kBrushless);

        ArrayList<CANSparkMax> sparkList = new ArrayList<CANSparkMax>() {
            {
                add(sparkA);
                add(sparkB);
                add(sparkC);
                add(sparkD);
            }
        };

        for (CANSparkMax spark : sparkList) {
            spark.setSecondaryCurrentLimit(maxCurrent);
            spark.setIdleMode(IdleMode.kBrake);
        }

        intakeSolenoid = new DoubleSolenoid(forwardPort, reversePort);

        this.operatorController = operatorController;
        this.tubeOutput = tubeOutput;
        this.intakeOutput = intakeOutput;
        this.threshold = threshold;
        intake = false;
    }

    public void run() {
        if (operatorController.getAButtonPressed()) {
            intake = !intake;
        }

        if (intake) {
            intakeDown();
        } else {
            intakeUp();
        }

        if (operatorController.getBumper(Hand.kLeft) || operatorController.getTriggerAxis(Hand.kLeft) >= threshold) {
            tubeIntake();
        } else if (operatorController.getTriggerAxis(Hand.kRight) >= threshold) {
            sparkC.set(1);
            sparkD.set(-1);
        } else {
            tubeOff();
        }

        if (operatorController.getTriggerAxis(Hand.kLeft) >= threshold) {
            intake();
        } else if (operatorController.getBumper(Hand.kRight)){
            intakeReverse();
        } else {
            intakeOff();
        }

    }

    public void intakeDown() {
        intakeSolenoid.set(Value.kForward);
    }

    public void intakeUp() {
        intakeSolenoid.set(Value.kReverse);
    }

    public void tubeIntake() {
        int constc = 1, constd = -1;

        sparkC.set(constc * tubeOutput);
        sparkD.set(constd * tubeOutput);
    }

    public void tubeOff() {
        sparkC.set(0);
        sparkD.set(0);
    }

    public void intake() {
        int consta = -1, constb = 1;

        sparkA.set(-1*consta * intakeOutput);
        sparkB.set(-1*constb * intakeOutput);
    }

    public void intakeReverse() {
        int consta = -1, constb = 1;

        sparkA.set(1 * consta * intakeOutput);
        sparkB.set(1 * constb * intakeOutput);
    }
    public void intakeOff() {
        sparkA.set(0);
        sparkB.set(0);
    }
}