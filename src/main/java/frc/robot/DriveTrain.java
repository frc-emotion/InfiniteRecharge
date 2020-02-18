package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;


/**
 * Class that runs the drive train
 * 
 * <p>
 * Autonomous:
 * <ul>
 * <li>update: Resets variables every autonomous cycle before running spinUp or
 * spinDown.</li>
 * <li>dashboardRun: Update SmartDashboard with the new variables</li>
 * <li>goToPose: Tries to move the robot to the give pose</li>
 * <li>interruptTrajectory: Interrupts current trajectory and stops motors</li>
 * </ul>
 * <p>
 * Operation: The left and right joystick on the drive controller will control
 * the left and right motors of the drive train respectively.
 * </p>
 * 
 * Note: teleopRun() will try to get driveExponent from the SmartDashboard and
 * will default to 1.8 if not found.
 * 
 * @author Ryan Chaiyakul
 */
public class DriveTrain {
    private XboxController driveController; // Controller checked during run()
    private CANSparkMax lsparkA, lsparkB, lsparkC, rsparkA, rsparkB, rsparkC; // Neos for the driveTrain
    private ArrayList<CANSparkMax> sparkList; // List of Neos from L to R and A to C
    private SpeedControllerGroup leftGroup, rightGroup; // SpeedController groups for each side
    private DifferentialDrive drive; // DifferentialDrive object
    private double slowPower, regularPower, turboPower; // Save power values

    public CANEncoder lEncoder, rEncoder;

    private PathConverter pathConverter;
    private boolean pathDone;

    private SendableChooser<Integer> driveChoices, pathChoices;

    public DriveTrain(int[] leftPorts, int[] rightPorts, int maxCurrent, double slowPower, double regularPower,
            double turboPower, XboxController driveController) {

        pathDone = false;
        // 3 Ports for each side
        if (leftPorts.length != 3 || rightPorts.length != 3) {
            return;
        }

        lsparkA = new CANSparkMax(leftPorts[0], MotorType.kBrushless);
        lsparkB = new CANSparkMax(leftPorts[1], MotorType.kBrushless);
        lsparkC = new CANSparkMax(leftPorts[2], MotorType.kBrushless);

        rsparkA = new CANSparkMax(rightPorts[0], MotorType.kBrushless);
        rsparkB = new CANSparkMax(rightPorts[1], MotorType.kBrushless);
        rsparkC = new CANSparkMax(rightPorts[2], MotorType.kBrushless);

        lEncoder = lsparkA.getEncoder();
        rEncoder = rsparkA.getEncoder();

        // Group sparks into an ArrayList for a cleaner intialization loop
        sparkList = new ArrayList<CANSparkMax>() {
            {
                add(lsparkA);
                add(lsparkB);
                add(lsparkC);
                add(rsparkA);
                add(rsparkB);
                add(rsparkC);
            }
        };

        for (CANSparkMax spark : sparkList) {
            spark.setSmartCurrentLimit(maxCurrent);
            spark.setSecondaryCurrentLimit(maxCurrent);
            spark.setIdleMode(IdleMode.kBrake); // IdleMode will always be kBrake for driveTrain motors
        }

        leftGroup = new SpeedControllerGroup(lsparkA, lsparkB, lsparkC);
        rightGroup = new SpeedControllerGroup(rsparkA, rsparkB, rsparkC);

        drive = new DifferentialDrive(leftGroup, rightGroup);

        // Store variables
        this.driveController = driveController;
        this.slowPower = slowPower;
        this.regularPower = regularPower;
        this.turboPower = turboPower;
    }

    /**
     * Function that should be called by teleopPeriodic
     */

    private void runPathFinderChoices() {
        if (!pathDone) {
            runPathFinder();
            pathDone = true;
        }
        if (pathConverter.isDriveAllowed())
            runTankDrive();
    }
    
    public void run() {
        //int driveChoice = driveChoices.getSelected();
        int driveChoice = 0;
        switch (driveChoice) {
        case 0:
            // Lets worry about this after drive train works
            runPathFinderChoices();
            break;
        case 1:
            runArcadeDrive();
        default:
            runTankDrive();
            break;
        }
    }

    /**
     * Function that sets the speeds for the DifferentialDrive object periodically
     */

    public CANEncoder getDriveEncoder(char side) {
        if (side == 'r')
            return rEncoder;
        else if (side == 'l')
            return lEncoder;
        else
            return null;
    }

    public void runPathFinder() {
        //int pathChoice = pathChoices.getSelected().intValue();
        String pathName = "";
        int pathChoice = 0;
        switch (pathChoice) {
        case 0:
            pathName = "RS1-B2";
            break;
        case 1:
            pathName = "straighthab";
            break;
        case 2:
            Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                    Trajectory.Config.SAMPLES_HIGH, 0.02, 0.4, 0.8, 5.0);
            Trajectory trajectory = Pathfinder.generate(PathTrajectories.rightHab, config);

            pathConverter = new PathConverter(this, trajectory);
            pathConverter.setUpFollowers();
            pathConverter.followPath();
            break;
        default:
            // do nothing
            break;
        }

        if (!pathName.equals("")) {
            String dir = Filesystem.getDeployDirectory().toString();
            String fileName = pathName + ".wpilib.csv";

            File trajFile = new File(dir + "/" + fileName);

            Trajectory traj = null;
            try {
                traj = Pathfinder.readFromCSV(trajFile);
                pathConverter = new PathConverter(this, traj);
                pathConverter.setUpFollowers();
                pathConverter.followPath();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private void runTankDrive() {
        // constants to easily configure if drive is opposite
        int constR = 1, constL = 1;

        // Get vertical value of the joysticks
        double rAxis = driveController.getY(Hand.kRight);
        double lAxis = driveController.getY(Hand.kLeft);

        // Use a constant multiplier for +/- direction as the driveExponent could be
        // even and negate the sign
        if (rAxis < 0) {
            constR *= 1;
        } else if (rAxis > 0) {
            constR *= -1;
        }

        if (lAxis < 0) {
            constL *= 1;
        } else if (lAxis > 0) {
            constL *= -1;
        }

        // LB and RB are used to change the drivePower on the fly
        double drivePower = regularPower;
        if (driveController.getBumper(Hand.kLeft))
            drivePower = slowPower;
        else if (driveController.getBumper(Hand.kRight))
            drivePower = turboPower;

        // However driveExponent should be constant (Changable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("driveExponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        double driveL = constL * drivePower * Math.pow(Math.abs(lAxis), driveExponent);
        double driveR = constR * drivePower * Math.pow(Math.abs(rAxis), driveExponent);

        // Our drivers prefer tankDrive
        drive.tankDrive(driveL, driveR);
    }

    /**
     * Function that should be called before autonomousPeriodic ends in autonomous.
     * Only exposed for autonomous in order to insure continually updates to
     * SmartDashboard.
     */
    public void dashboardRun() {
        double[] motorTemps = new double[6];
        for (int i = 0; i < motorTemps.length; i++) {
            motorTemps[i] = sparkList.get(i).getMotorTemperature();
        }

        SmartDashboard.putNumberArray("DriveTrainTemperature(lA|lB|lC|rA|rB|rC)", motorTemps);
    }

    public DifferentialDrive getDrive() {
        return drive;
    }

    void runArcadeDrive() {
        // arcade drive (one stick) with square inputs
        drive.arcadeDrive(Robot.driveController.getY(Hand.kLeft), Robot.driveController.getX(Hand.kLeft), true);
    }
}