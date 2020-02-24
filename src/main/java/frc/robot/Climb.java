package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import edu.wpi.first.wpilibj.DigitalInput;


public class Climb {
    SpeedControllerGroup climbGroup;
    private XboxController operatorController;
    WPI_TalonSRX screwTalonL, screwTalonR;
    DoubleSolenoid pistonL, pistonR;
    private static int pistonPos = 0;
    public static DigitalInput upperClimbLimitSwitch;
    public static DigitalInput lowerClimbLimitSwitch;


    double deadzone = 0.1;

    public Climb(XboxController operatorController) {
        workShuffleBoard();

        upperClimbLimitSwitch = new DigitalInput(Constants.upperClimbLimitPort);
        lowerClimbLimitSwitch = new DigitalInput(Constants.lowerClimbLimitPort);
        
        screwTalonL = new WPI_TalonSRX(Constants.SCREW_TALON_L);
        screwTalonR = new WPI_TalonSRX(Constants.SCREW_TALON_R);
        screwTalonL.setNeutralMode(NeutralMode.Brake);
        screwTalonR.setNeutralMode(NeutralMode.Brake);

        climbGroup = new SpeedControllerGroup(screwTalonL, screwTalonR);

        pistonL = new DoubleSolenoid(Constants.PISTONL_FWD, Constants.PISTONL_BKWD);
        pistonR = new DoubleSolenoid(Constants.PISTONR_FWD, Constants.PISTRONR_BKWD);

        screwTalonL.configContinuousCurrentLimit(Constants.kTalonMaxCurrent);
        

        screwTalonR.configContinuousCurrentLimit(Constants.kTalonMaxCurrent);

        this.operatorController = operatorController;

    }

    public void run() {
        runPistons();
        runScrews();
        workShuffleBoard();
    }

    public void runPistons() {
        if (operatorController.getYButtonPressed()) {
            if (pistonPos == 0) {
                pistonL.set(Value.kForward);
                pistonR.set(Value.kForward);
                pistonPos = 1;
            } else if (pistonPos == 1) {
                pistonL.set(Value.kReverse);
                pistonR.set(Value.kReverse);
                pistonPos = 0;
            }
        }
    }

    public void runScrews() {
        double hand = operatorController.getY(Hand.kRight);
       /* if((upperClimbLimitSwitch.get() && hand > 0) || (lowerClimbLimitSwitch.get() && hand<0)) {
            climbGroup.set(0);
        } else {*/
            if (Math.abs(operatorController.getY(Hand.kRight)) >= deadzone) {
                climbGroup.set(hand * Constants.SCREW_SPEED);
            } else {
                climbGroup.set(0);
            }
        }
    //}

   


    public void workShuffleBoard() {
        SmartDashboard.putNumber("Piston Position", pistonPos);
       /*SmartDashboard.putNumber("TalonL Current", screwTalonL.getStatorCurrent());
        SmartDashboard.putNumber("TalonR Current", screwTalonR.getStatorCurrent());
        SmartDashboard.putNumber("TalonL Voltage", screwTalonL.getMotorOutputVoltage());
        SmartDashboard.putNumber("TalonR Voltage", screwTalonR.getMotorOutputVoltage());
*/
    }
}