package frc.robot.launcher;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Intake {
    private XboxController operatorController;
    private CANSparkMax sparkA, sparkB, sparkC, sparkD;
    private SpeedControllerGroup tubeGroup;
    private SpeedControllerGroup intakeGroup;

    private double tubeOutput;
    private double intakeOutput;
    private double threshold;

    public Intake(int[] ports, int maxCurrent, double intakeOutput, double tubeOutput, double threshold, XboxController operatorController) {
        if (ports.length != 2) {
            return;
        }

        sparkA = new CANSparkMax(ports[0], MotorType.kBrushless);
        sparkB = new CANSparkMax(ports[1], MotorType.kBrushless);

        ArrayList<CANSparkMax> sparkList = new ArrayList<CANSparkMax>() {
            {
                add(sparkA);
                add(sparkB);
            }
        };

        for (CANSparkMax spark : sparkList) {
            spark.setSecondaryCurrentLimit(maxCurrent);
            spark.setIdleMode(IdleMode.kBrake);
        }

        this.operatorController = operatorController;
        this.tubeOutput = tubeOutput;
        this.intakeOutput = intakeOutput;
        this.threshold = threshold;
    }

    public void run() {
        if (operatorController.getAButton()) {

        }

        if (operatorController.getBumper(Hand.kLeft)) {
            tubeIntake();
        } else {
            tubeGroup.set(0);
        }

        if (operatorController.getTriggerAxis(Hand.kLeft) >= threshold) {
            intake();
        } else {
            intakeGroup.set(0);
        }

    }

    public void tubeDown() {

    }

    public void tubeUp() {

    }

    public void tubeIntake() {
        int constc = 1, constd = 1;

        sparkC.set(constc * tubeOutput);
        sparkD.set(constd * tubeOutput);
    }

    public void intake() {
        int consta = 1, constb = 1;

        sparkA.set(consta * intakeOutput);
        sparkB.set(constb * intakeOutput);
    }

    public void stop() {
        intakeGroup.set(0);
    }

    public void dashboardRun() {

    }
}