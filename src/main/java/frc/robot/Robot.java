/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.launcher.*;
import frc.robot.drivetrain.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public XboxController operatorController;
  public XboxController driveController;

  private Shooter shooter;
  private DriveTrain drive;
  private Intake intake;
  private AutoOptions auto;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    operatorController = new XboxController(Constants.kOperatorPort);
    driveController = new XboxController(Constants.kDrivePort);

    auto = new AutoOptions(Constants.kStartingLocations, Constants.kValidColors);

    // Shooter Initialization
    shooter = new Shooter(Constants.kShooterPorts, Constants.kShooterForwardPort, Constants.kShooterReversePort,
        Constants.kSparkMaxCurrent, Constants.kShooterRPM, Constants.kShooterWaitTime, Constants.kShooterThresholdRPM,
        Constants.kShooterThresholdTrigger, Constants.kShooterMaxOutput, operatorController);

    // Get Shooter Controller
    SendableChooser<String> shooterSelector = new SendableChooser<>();
    shooterSelector.setDefaultOption("TwoStep", Constants.kTwoStep);
    shooterSelector.addOption("PID", Constants.kPID);
    SmartDashboard.putData(shooterSelector);

    switch (shooterSelector.getSelected()) {
    case Constants.kTwoStep:
      shooter.enableTwoStepController(Constants.kShooterMinOutput);
    case Constants.kPID:
      shooter.enablePIDController(Constants.kPShooter, Constants.kIShooter, Constants.kDShooter);
    }

    shooterSelector.close();

    shooter.enablePivot(Constants.kPivotPort, Constants.kSparkMaxCurrent, Constants.kPivotLowerLimitPort,
        Constants.kTeleopConstant, Constants.kCallibrateSpeed, Constants.kRevToAngle,
        Constants.kPivotControllerThreshold, Constants.kPivotAngleThreshold, Constants.kPivotMaxAngle,
        Constants.kMountingHeight, Constants.kMountingAngle, Constants.kRefrenceHeight, Constants.kPortPipeline,
        Constants.kShooterMaxVelocity);
    shooter.enablePIDController(Constants.kPivotP, Constants.kPivotI, Constants.kPivotD);

    intake = new Intake(Constants.kIntakePorts, Constants.kSparkMaxCurrent, Constants.kIntakeForwardPort,
        Constants.kIntakeReversePort, Constants.kIntakeOutput, Constants.kTubeOutput, Constants.kIntakeThreshold,
        operatorController);

    drive = new DriveTrain(Constants.kDriveLeftPorts, Constants.kDriveRightPorts, Constants.kSparkMaxCurrent,
        Constants.kSlowPower, Constants.kRegularPower, Constants.kTurboPower, driveController);
    drive.enablePIDControl(Constants.kPDrive, Constants.kIDrive, Constants.kDDrive, Constants.kDriveTolerance,
        Constants.kDriveRotation);
    drive.enableAlignment(Constants.kPortPipeline);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    shooter.update();

    shooter.dashboardRun();
    drive.dashboardRun();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    shooter.run();
    drive.run();
    intake.run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
