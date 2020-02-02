package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Intake {
    private XboxController operatorController;
    private CANSparkMax sparkA, sparkB;
    private SpeedControllerGroup intakeGroup;

    private double maxOutput;
    private double threshold;

    Intake(int[] ports, int maxCurrent, double maxOutput, double threshold, XboxController operatorController) {
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
            spark.set(maxCurrent);
            spark.setIdleMode(IdleMode.kBrake);
        }

        this.operatorController = operatorController;
        this.maxOutput = maxOutput;
        this.threshold = threshold;
    }

    public void run() {
        if (operatorController.getAButton()) {

        } else if (operatorController.getBumper(Hand.kLeft)) {

        } else if (operatorController.getTriggerAxis(Hand.kLeft) >= threshold) {

        } else {
            stop();
        }

    }

    public void intake() {
        int consta = 1, constb = 1;

        sparkA.set(consta * maxOutput);
        sparkB.set(constb * maxOutput);
    }

    public void stop() {
        intakeGroup.set(0);
    }

    public void dashboardRun() {

    }
}