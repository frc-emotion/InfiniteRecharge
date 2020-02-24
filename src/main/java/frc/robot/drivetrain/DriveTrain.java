package frc.robot.drivetrain;

import java.util.ArrayList;

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
import frc.robot.PIDControl;
import frc.robot.PathConverter;
import frc.robot.PathTrajectories;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;

/**
 * 
 * /** Class that runs the drive train
 * 
 * <p>
 * Autonomous:
 * <ul>
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
    private boolean invert; // Flag for whether directions are inverted
    private boolean tester = true;
    private Alignment alignment;

    private PIDControl pidControl;

    private SendableChooser<Integer> driveChoices, pathChoices;
    private PathConverter pathConverter;
    private boolean pathDone;

    public CANEncoder rEnc, lEnc;

    private double curTime;

    public DriveTrain(int[] leftPorts, int[] rightPorts, int maxCurrent, double slowPower, double regularPower,
            double turboPower, XboxController driveController) {
        initShuffleBoard();

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
        drive.setSafetyEnabled(false);
        // Store variables//
        this.driveController = driveController;
        this.slowPower = slowPower;
        this.regularPower = regularPower;
        this.turboPower = turboPower;
        this.invert = false;
    }

    void runArcadeDrive() {
        // arcade drive (one stick) with square inputs
        drive.arcadeDrive(driveController.getY(Hand.kLeft), driveController.getX(Hand.kLeft), true);
    }

    public void enablePIDControl(float kP, float kI, float kD, double tolerance, double maxRotation) {
        pidControl = new PIDControl(kP, kI, kD);

        pidControl.setTolerance(tolerance);
        pidControl.setMaxSpeed(maxRotation);
    }

    public void enableAlignment(int pipeline) {
        alignment = new Alignment(pipeline);
    }

    public void align() {
        if (alignment.targetFound()) {
            drive.arcadeDrive(0, pidControl.getValue(0, alignment.getError()));
        }
    }

    public CANEncoder getEncoder(char selector) {
        switch (selector) {
            case 'l':
                return lsparkA.getEncoder();
            case 'r':
                return rsparkA.getEncoder();
        }
        return null;
    }

    public DifferentialDrive getDrive() {
        return drive;
    }

    /**
     * Function that should be called by teleopPeriodic
     */

    public void autoChoice1() {

        if (tester == true) {
            curTime = System.currentTimeMillis();

            while (System.currentTimeMillis() - curTime < 500) {
                drive.tankDrive(0.5, 0.5);
                tester = false;
            }

        }
        if (tester == false) {
            drive.tankDrive(0, 0);
        }

    }

    public void run() {

        if (driveController.getAButtonPressed()) {
            invert = !invert;
        }

        if (driveController.getBButton()) {
            align();
        } else {
            runTankDrive();
            pidControl.cleanup();
        }

        dashboardRun();
    }

    public void runPathFinderChoices() {
        System.out.println(pathDone);
        if (!pathDone) {
            runPathFinder();
            pathDone = true;
        } else {
            return;
        }
    }

    public void runPathFinder() {

        int pathChoice = pathChoices.getSelected().intValue();

        String pathName = "";

        switch (pathChoice) {
            case 0:
                pathName = "slowStraight.pf1";
                break;
            case 1:
                pathName = "RS1-B2.wpilib";
                break;
            case 2:
                pathName = "Straight.pf1";
            case 10:
                Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                        Trajectory.Config.SAMPLES_HIGH, 0.02, 0.4, 0.8, 5.0);
                Trajectory trajectory = Pathfinder.generate(PathTrajectories.rightHab, config);

                pathConverter = new PathConverter(this, driveController, trajectory);
                pathConverter.setUpFollowers();
                pathConverter.followPath();
                break;
            default:
                // do nothing
                break;
        }

        if (!pathName.equals("")) {
            String dir = Filesystem.getDeployDirectory().toString();
            String fileName = pathName + ".csv";

            File trajFile = new File(dir + "/" + fileName);
            System.out.println(trajFile);

            Trajectory traj = null;
            try {
                traj = Pathfinder.readFromCSV(trajFile);
                pathConverter = new PathConverter(this, driveController, traj);
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

        // However driveExponent should be constant (Changeable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("Drive Exponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        double driveL = constL * drivePower * Math.pow(Math.abs(lAxis), driveExponent);
        double driveR = constR * drivePower * Math.pow(Math.abs(rAxis), driveExponent);

        // Our drivers prefer tankDrive
        // invert will switch R and L
        if (invert) {
            drive.tankDrive(-driveR, -driveL);
        } else {
            drive.tankDrive(driveL, driveR);
        }
    }

    /**
     * Function that should be called before autonomousPeriodic ends in autonomous.
     * Only exposed for autonomous in order to insure continually updates to
     * SmartDashboard.
     */

    public void autoDrive() {
        drive.tankDrive(0.2, 0.2);
    }

    private void initShuffleBoard() {

        SmartDashboard.putBoolean("Pathfinder Job", false);

        pathChoices = new SendableChooser<Integer>();
        pathChoices.setDefaultOption("slowStraight", 0);

        pathChoices.addOption("RS1-B2", 1);
        pathChoices.addOption("slowStraight", 0);
        pathChoices.addOption("Straight", 2);
        SmartDashboard.putData("PathFinder Choices", pathChoices);
    }

    public void dashboardRun() {
        double[] motorTemps = new double[6];
        for (int i = 0; i < motorTemps.length; i++) {
            motorTemps[i] = sparkList.get(i).getMotorTemperature();
        }

        SmartDashboard.putNumberArray("DriveTrainTemperature(lA|lB|lC|rA|rB|rC)", motorTemps);
        SmartDashboard.putBoolean("Aligned", pidControl.isInRange());
        SmartDashboard.putBoolean("PortFound", alignment.targetFound());

    }
}