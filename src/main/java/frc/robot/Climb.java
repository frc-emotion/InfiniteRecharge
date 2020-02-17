package frc.robot;



import com.ctre.phoenix.motorcontrol.ControlMode;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Climb {    
    WPI_TalonSRX screwTalonL, screwTalonR;
    DoubleSolenoid pistonL, pistonR;
    private static int pistonPos = 0;

    double deadzone = 0.1;

    public Climb() {

        screwTalonL = new WPI_TalonSRX(Constants.SCREW_TALON_L);
        screwTalonR = new WPI_TalonSRX(Constants.SCREW_TALON_R);

        pistonL = new DoubleSolenoid(Constants.PISTONL_FWD,Constants.PISTONL_BKWD);
        pistonR = new DoubleSolenoid(Constants.PISTONR_FWD,Constants.PISTRONR_BKWD);

        screwTalonL.configContinuousCurrentLimit(70);
        
        screwTalonR.configContinuousCurrentLimit(70);


    }

    public void run() {
        runPistons();
        runScrews();
        workShuffleBoard();
    }

    public void runPistons() {
        if(Robot.operatorController.getYButtonPressed()) {
            if(pistonPos == 0){
                pistonL.set(Value.kForward);
                pistonR.set(Value.kForward);
                pistonPos = 1;
            } else if(pistonPos == 1) {
                pistonL.set(Value.kReverse);
                pistonR.set(Value.kReverse);
                pistonPos = 0;
            }
        }
    }

    public void runScrews() {
        if(Math.abs(Robot.operatorController.getY(Hand.kRight)) >= deadzone)  {
            screwTalonL.set(ControlMode.PercentOutput, Constants.SCREW_SPEED * Robot.operatorController.getY(Hand.kRight));
            screwTalonR.set(ControlMode.PercentOutput, Constants.SCREW_SPEED * Robot.operatorController.getY(Hand.kRight));
        } else {
            screwTalonL.set(ControlMode.PercentOutput, 0);
            screwTalonR.set(ControlMode.PercentOutput, 0);
        }
    }

    public void workShuffleBoard() {
        SmartDashboard.putNumber("Piston Position", pistonPos);



    }
}