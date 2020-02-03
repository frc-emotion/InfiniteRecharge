package frc.robot.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

class DriveController {
    private CANEncoder lEncoder, rEncoder;
    private CANPIDController lController, rController;
    private SpeedControllerGroup lGroup, rGroup;

    DriveController(CANEncoder lEncoder, CANEncoder rEncoder, CANPIDController lController,
            CANPIDController rController, SpeedControllerGroup lGroup, SpeedControllerGroup rGroup) {
        this.lEncoder = lEncoder;
        this.rEncoder = rEncoder;
        this.lController = lController;
        this.rController = rController;
        this.lGroup = lGroup;
        this.rGroup = rGroup;
    }

    private CANPIDController getController(char selector) {
        switch (selector) {
        case 'l':
            return lController;
        case 'r':
            return rController;
        default:
            return null;
        }
    }

    public void setKp(char selector, double value) {
        getController(selector).setP(value);
    }

    public void setKi(char selector, double value) {
        getController(selector).setI(value);
    }

    public void setKd(char selector, double value) {
        getController(selector).setD(value);
    }

    public void setRefrence(char selector, double value, ControlType controlType) {
        getController(selector).setReference(value, controlType);
    }
}