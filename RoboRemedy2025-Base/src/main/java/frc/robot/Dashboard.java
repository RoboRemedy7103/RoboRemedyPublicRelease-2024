// Dashboard.java - Controls the Shuffleboard dashboard
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.*;
import frc.robot.RobotEnums.*;

import java.util.Map;

public class Dashboard {
    private RoboLog rLog;
    private Electronics e;
    private RobotState robotState;
    private Action act;
    private OI oi;

    // Teleop Tab
    private GenericEntry isBothJoystick;
    private GenericEntry driverStick;
    private GenericEntry operatorStick;
    private GenericEntry robotOK;

    // Test Tab
    private SendableChooser<TestMode> testChooser;
    private GenericEntry testInstructions;
    private GenericEntry testString;
    private GenericEntry deadReckoningX;
    private GenericEntry deadReckoningY;
    private GenericEntry lidarDistance;
    private GenericEntry pivotAngleTest;

    // Autonomous Tab
    private SendableChooser<AutoProgram> autoProgram;
    private SendableChooser<AutoStartPosition> autoStartingPosition;

    // PreGame Tab
    private GenericEntry frontLeftDrive;
    private GenericEntry frontLeftTurn;
    private GenericEntry frontLeftCancoder;
    private GenericEntry frontRightDrive;
    private GenericEntry frontRightTurn;
    private GenericEntry frontRightCancoder;
    private GenericEntry backLeftDrive;
    private GenericEntry backLeftTurn;
    private GenericEntry backLeftCancoder;
    private GenericEntry backRightDrive;
    private GenericEntry backRightTurn;
    private GenericEntry backRightCancoder;
    private GenericEntry doSwerveEncodersMatch;
    private GenericEntry isSampleMotorGood;
    private GenericEntry isPivotMotorGood;
    private GenericEntry isPivotCancoderGood;
    private GenericEntry isMainAprilTagCameraAttached;
    private GenericEntry isGamePieceCameraAttached;
    private GenericEntry isBattery;

    // Joystick Tab
    private GenericEntry zeroJoystickName;
    private GenericEntry zeroJoystickType;
    private GenericEntry zeroJoystickStatus;
    private GenericEntry oneJoystickName;
    private GenericEntry oneJoystickType;
    private GenericEntry oneJoystickStatus;
    private GenericEntry twoJoystickName;
    private GenericEntry twoJoystickType;
    private GenericEntry twoJoystickStatus;
    private GenericEntry threeJoystickName;
    private GenericEntry threeJoystickType;
    private GenericEntry threeJoystickStatus;
    private GenericEntry fourJoystickName;
    private GenericEntry fourJoystickType;
    private GenericEntry fourJoystickStatus;
    private GenericEntry fiveJoystickName;
    private GenericEntry fiveJoystickType;
    private GenericEntry fiveJoystickStatus;
    private GenericEntry driverTestButton;
    private GenericEntry operatorTestButton;
    private GenericEntry driverName;
    private GenericEntry operatorName;

    Dashboard(Electronics e, RoboLog rLog, Action act, OI oi, RobotState robotState) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;

        if (this.rLog == null) {
            System.out.println("Warning: Dashboard.rLog is null");
        }
        if (this.e == null) {
                rLog.print("Warning: Dashboard.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: Dashboard.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: Dashboard.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: Dashboard.robotState is null");
        }
    
        InitTelopTab();
        InitTestTab();
        InitAutonomousTab();
        InitPreGameTab();
        InitJoystickTab();
    }

    public void selectTab(String selectTab) {
        rLog.print("Selecting new tab: " + selectTab);
        Shuffleboard.selectTab(selectTab);
    }

    public void InitTelopTab() {
        Shuffleboard.getTab("Teleop")
                .add("Robot Name", e.getRobotName())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0)
                .getEntry();

        Shuffleboard.getTab("Teleop")
                .add("Gyro", e.gyro)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(7, 0)
                .withSize(2, 2);

        driverStick = Shuffleboard.getTab("Teleop")
                .add("Driver Joystick", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "white"))
                .withPosition(3, 1)
                .withSize(1, 1)
                .getEntry();

        operatorStick = Shuffleboard.getTab("Teleop")
                .add("Operator Joystick", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "white"))
                .withPosition(4, 1)
                .withSize(1, 1)
                .getEntry();

        robotOK = Shuffleboard.getTab("Teleop")
                .add("Robot Ok", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(5, 2)
                .getEntry();

        isBothJoystick = Shuffleboard.getTab("Teleop")
                .add("Both Joystick", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(2, 1)
                .getEntry();
    }

    public void InitTestTab() {
        testChooser = new SendableChooser<TestMode>();
        testChooser.setDefaultOption("Quick Test", TestMode.QuickTest);
        testChooser.addOption("Swerve Test", TestMode.SwerveTest);
        testChooser.addOption("Swerve Drive", TestMode.SwerveDrive);
        testChooser.addOption("Test Facing", TestMode.TestFacing);
        testChooser.addOption("Test Motors", TestMode.TestMotors);
        testChooser.addOption("Test Motors With Joystick", TestMode.TestMotors);
        testChooser.addOption("Tune Motor", TestMode.TuneMotor);
        testChooser.addOption("Test LED Regions", TestMode.TestLEDRegions);
        testChooser.addOption("Test LED Colors", TestMode.TestLEDColors);
        testChooser.addOption("Reset Swerve Encoders", TestMode.ResetSwerveEncoders);
        testChooser.addOption("Reset Pivot Encoder", TestMode.ResetPivotEncoder);
        testChooser.addOption("Test Countdown", TestMode.TestCountdown);
        testChooser.addOption("Slow Pivot", TestMode.SlowPivot);
        testChooser.addOption("Aim Pivot", TestMode.AimPivot);
        testChooser.addOption("Test None", TestMode.TestNone);

        Shuffleboard.getTab("Test")
                .add("Test Mode", testChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        Shuffleboard.getTab("Test")
                .add("Gyro Deg", e.gyro)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(7, 0)
                .withSize(2, 2);

        Shuffleboard.getTab("Test")
                .add("Robot Name", e.getRobotName())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0)
                .withSize(1, 1)
                .getEntry();

        testInstructions = Shuffleboard.getTab("Test")
                .add("Test Instructions", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 1)
                .withSize(3, 1)
                .getEntry();

        testString = Shuffleboard.getTab("Test")
                .add("Test String", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 2)
                .withSize(3, 1)
                .getEntry();

        deadReckoningX = Shuffleboard.getTab("Test")
                .add("Dead Reckoning X", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        deadReckoningY = Shuffleboard.getTab("Test")
                .add("Dead Reckoning Y", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(3, 0)
                .withSize(1, 1)
                .getEntry();

        lidarDistance = Shuffleboard.getTab("Test")
                .add("Lidar Distance", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();

        pivotAngleTest = Shuffleboard.getTab("Test")
                .add("Pivot Angle", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(5, 2)
                .withSize(1, 1)
                .getEntry();
    }

    public void InitAutonomousTab() {
        autoProgram = new SendableChooser<AutoProgram>();
        autoProgram.setDefaultOption("Nothing", AutoProgram.None);
        autoProgram.addOption("Just Align", AutoProgram.JustAlign);
        autoProgram.addOption("Drive Out", AutoProgram.DriveOut);
        autoProgram.addOption("Simulation Project", AutoProgram.SimulationProject);

        autoStartingPosition = new SendableChooser<AutoStartPosition>();
        autoStartingPosition.setDefaultOption("Left", AutoStartPosition.Left);
        autoStartingPosition.addOption("Right", AutoStartPosition.Right);

        Shuffleboard.getTab("Autonomous")
                .add("Auto Program", autoProgram)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 1)
                .withSize(2, 1);

        Shuffleboard.getTab("Autonomous")
                .add("Starting Position", autoStartingPosition)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        Shuffleboard.getTab("Autonomous")
                .add("Robot Name", e.getRobotName())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0)
                .withSize(1, 1)
                .getEntry();

        Shuffleboard.getTab("Autonomous")
                .add("Gyro", e.gyro)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(7, 0)
                .withSize(2, 2);
    }

    public void InitPreGameTab() {
        frontLeftDrive = Shuffleboard.getTab("PreGame")
                .add("fL Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(0, 0)
                .getEntry();

        frontLeftTurn = Shuffleboard.getTab("PreGame")
                .add("fL Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(0, 1)
                .getEntry();

        frontRightDrive = Shuffleboard.getTab("PreGame")
                .add("fR Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(1, 0)
                .getEntry();

        frontRightTurn = Shuffleboard.getTab("PreGame")
                .add("fR Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(1, 1)
                .getEntry();

        backLeftDrive = Shuffleboard.getTab("PreGame")
                .add("bL Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(2, 0)
                .getEntry();

        backLeftTurn = Shuffleboard.getTab("PreGame")
                .add("bL Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(2, 1)
                .getEntry();

        backRightDrive = Shuffleboard.getTab("PreGame")
                .add("bR Drive", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(3, 0)
                .getEntry();

        backRightTurn = Shuffleboard.getTab("PreGame")
                .add("bR Turn", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(3, 1)
                .getEntry();

        frontLeftCancoder = Shuffleboard.getTab("PreGame")
                .add("fL Cancoder", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        frontRightCancoder = Shuffleboard.getTab("PreGame")
                .add("fR Cancoder", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();

        backLeftCancoder = Shuffleboard.getTab("PreGame")
                .add("bL Cancoder", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        backRightCancoder = Shuffleboard.getTab("PreGame")
                .add("bR Cancoder", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(3, 2)
                .withSize(1, 1)
                .getEntry();

        doSwerveEncodersMatch = Shuffleboard.getTab("PreGame")
                .add("Swerve Encoders", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(7, 2)
                .getEntry();

        Shuffleboard.getTab("PreGame")
                .add("Robot Name", e.getRobotName())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0)
                .getEntry();

        Shuffleboard.getTab("PreGame")
                .add("Gyro Deg", e.gyro)
                .withPosition(7, 0)
                .withWidget(BuiltInWidgets.kGyro)// kDial
                .withSize(2, 2);

        isBattery = Shuffleboard.getTab("PreGame")
                .add("Battery OK", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(6, 2)
                .getEntry();

        isMainAprilTagCameraAttached = Shuffleboard.getTab("PreGame")
                .add("AprilTagCam", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(4, 2)
                .getEntry();

        isGamePieceCameraAttached = Shuffleboard.getTab("PreGame")
                .add("GamePieceCam", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(5, 2)
                .getEntry();

        isSampleMotorGood = Shuffleboard.getTab("PreGame")
                .add("Sample Motor", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(4, 0)
                .withSize(1, 1)
                .getEntry();

        isPivotMotorGood = Shuffleboard.getTab("PreGame")
                .add("Pivot Motor", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(5, 0)
                .withSize(1, 1)
                .getEntry();

        isPivotCancoderGood = Shuffleboard.getTab("PreGame")
                .add("Pivot Cancoder", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(6, 1)
                .withSize(1, 1)
                .getEntry();
    }

    public void InitJoystickTab() {
        zeroJoystickName = Shuffleboard.getTab("Joystick")
                .add("Joystick 0 Name", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();

        zeroJoystickStatus = Shuffleboard.getTab("Joystick")
                .add("Joystick 0 Status", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();

        zeroJoystickType = Shuffleboard.getTab("Joystick")
                .add("Joystick 0 Type", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        oneJoystickName = Shuffleboard.getTab("Joystick")
                .add("Joystick 1 Name", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();

        oneJoystickStatus = Shuffleboard.getTab("Joystick")
                .add("Joystick 1 Status", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();

        oneJoystickType = Shuffleboard.getTab("Joystick")
                .add("Joystick 1 Type", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();

        twoJoystickName = Shuffleboard.getTab("Joystick")
                .add("Joystick 2 Name", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        twoJoystickStatus = Shuffleboard.getTab("Joystick")
                .add("Joystick 2 Status", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();

        twoJoystickType = Shuffleboard.getTab("Joystick")
                .add("Joystick 2 Type", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        threeJoystickName = Shuffleboard.getTab("Joystick")
                .add("Joystick 3 Name", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(4, 0)
                .withSize(1, 1)
                .getEntry();

        threeJoystickStatus = Shuffleboard.getTab("Joystick")
                .add("Joystick 3 Status", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(5, 0)
                .withSize(1, 1)
                .getEntry();

        threeJoystickType = Shuffleboard.getTab("Joystick")
                .add("Joystick 3 Type", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0)
                .withSize(1, 1)
                .getEntry();

        fourJoystickName = Shuffleboard.getTab("Joystick")
                .add("Joystick 4 Name", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(4, 1)
                .withSize(1, 1)
                .getEntry();

        fourJoystickStatus = Shuffleboard.getTab("Joystick")
                .add("Joystick 4 Status", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();

        fourJoystickType = Shuffleboard.getTab("Joystick")
                .add("Joystick 4 Type", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 1)
                .withSize(1, 1)
                .getEntry();

        fiveJoystickName = Shuffleboard.getTab("Joystick")
                .add("Joystick 5 Name", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(4, 2)
                .withSize(1, 1)
                .getEntry();

        fiveJoystickStatus = Shuffleboard.getTab("Joystick")
                .add("Joystick 5 Status", "")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(5, 2)
                .withSize(1, 1)
                .getEntry();

        fiveJoystickType = Shuffleboard.getTab("Joystick")
                .add("Joystick 5 Type", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 2)
                .withSize(1, 1)
                .getEntry();

        driverTestButton = Shuffleboard.getTab("Joystick")
                .add("Driver Test", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(8, 0)
                .withSize(1, 1)
                .getEntry();

        operatorTestButton = Shuffleboard.getTab("Joystick")
                .add("Operator Test", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "green", "color when false", "red"))
                .withPosition(9, 0)
                .withSize(1, 1)
                .getEntry();

        driverName = Shuffleboard.getTab("Joystick")
                .add("Driver Name", "Controller (Xbox One For Windows)")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(8, 1)
                .withSize(1, 1)
                .getEntry();

        operatorName = Shuffleboard.getTab("Joystick")
                .add("Operator Name", "Controller (Xbox360 Controller for Windows)")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(9, 1)
                .withSize(1, 1)
                .getEntry();
    }

    public void dashboardPeriodic() {
        //
        // Entries for Teleop Tab
        //
        displayBoolean(isBothJoystick, DriverStation.isJoystickConnected(1) && DriverStation.isJoystickConnected(0)
                && DriverStation.getJoystickIsXbox(0) && DriverStation.getJoystickIsXbox(1));
        displayBoolean(driverStick, oi.isJoystickButtonPressed(0, 1));
        displayBoolean(operatorStick, oi.isJoystickButtonPressed(1, 1));
        displayBoolean(robotOK, robotState.isRobotOk);

        //
        // Entries for Test Tab
        //
        displayDoubleRound3(deadReckoningX, robotState.robotPosition.getX());
        displayDoubleRound3(deadReckoningY, robotState.robotPosition.getY());
        if (e.isLidarCameraConnected()) {
            displayDoubleRound2(lidarDistance, e.getLidarDistance());
        } else {
            displayString(lidarDistance, "");
        }
        displayDoubleRound2(pivotAngleTest, robotState.pivotAngle);

        //
        // Entries for Autonomous Tab
        //

        //
        // Entries for Pre-Game Tab
        //
        // Pre-Game Swerve
        displayBoolean(frontLeftDrive, robotState.isFLDriveGood);
        displayBoolean(frontLeftTurn, robotState.isFLTurnGood);
        displayBoolean(frontLeftCancoder, robotState.isFLCancoderGood);
        displayBoolean(frontRightDrive, robotState.isFRDriveGood);
        displayBoolean(frontRightTurn, robotState.isFRTurnGood);
        displayBoolean(frontRightCancoder, robotState.isFRCancoderGood);
        displayBoolean(backLeftDrive, robotState.isBLDriveGood);
        displayBoolean(backLeftTurn, robotState.isBLTurnGood);
        displayBoolean(backLeftCancoder, robotState.isBLCancoderGood);
        displayBoolean(backRightDrive, robotState.isBRDriveGood);
        displayBoolean(backRightTurn, robotState.isBRTurnGood);
        displayBoolean(backRightCancoder, robotState.isBRCancoderGood);
        displayBoolean(doSwerveEncodersMatch, robotState.doSwerveTurnEncodersMatch);
        // Pre-Game Others
        displayBoolean(isSampleMotorGood, robotState.isSampleMotorGood);
        displayBoolean(isPivotMotorGood, robotState.isPivotMotorGood);
        displayBoolean(isPivotCancoderGood, robotState.isPivotEncoderGood);
        displayBoolean(isMainAprilTagCameraAttached, robotState.isMainAprilTagCameraAttached);
        displayBoolean(isGamePieceCameraAttached, robotState.isGamePieceCameraAttached);
        displayBoolean(isBattery, robotState.isBatteryGood);

        //
        // Entries for Joystick Tab
        //
        displayString(zeroJoystickName, oi.Joysticks[0].getJoystickName());
        displayString(zeroJoystickStatus, oi.Joysticks[0].getJoystickStatus());
        displayInteger(zeroJoystickType, oi.Joysticks[0].getJoystickType());
        displayString(oneJoystickName, oi.Joysticks[1].getJoystickName());
        displayString(oneJoystickStatus, oi.Joysticks[1].getJoystickStatus());
        displayInteger(oneJoystickType, oi.Joysticks[1].getJoystickType());
        displayString(twoJoystickName, oi.Joysticks[2].getJoystickName());
        displayString(twoJoystickStatus, oi.Joysticks[2].getJoystickStatus());
        displayInteger(twoJoystickType, oi.Joysticks[2].getJoystickType());
        displayString(threeJoystickName, oi.Joysticks[3].getJoystickName());
        displayString(threeJoystickStatus, oi.Joysticks[3].getJoystickStatus());
        displayInteger(threeJoystickType, oi.Joysticks[3].getJoystickType());
        displayString(fourJoystickName, oi.Joysticks[4].getJoystickName());
        displayString(fourJoystickStatus, oi.Joysticks[4].getJoystickStatus());
        displayInteger(fourJoystickType, oi.Joysticks[4].getJoystickType());
        displayString(fiveJoystickName, oi.Joysticks[5].getJoystickName());
        displayString(fiveJoystickStatus, oi.Joysticks[5].getJoystickStatus());
        displayInteger(fiveJoystickType, oi.Joysticks[5].getJoystickType());
        displayBoolean(driverTestButton, oi.isDriverTestButtonPressed());
        displayBoolean(operatorTestButton, oi.isOperatorTestButtonPressed());
        oi.changePreferredDriverName(driverName.getString("Controller (Xbox One For Windows)"));
        oi.changePreferredOperatorName(operatorName.getString("Controller (Xbox360 Controller for Windows"));
    }

    public TestMode getTestMode() {
        if (testChooser != null)
            return testChooser.getSelected();
        else
            return TestMode.TestNone;
    }

    public AutoProgram geAutoProgram() {
        if (autoProgram != null)
            return autoProgram.getSelected();
        else
            return AutoProgram.None;
    }

    public AutoStartPosition getAutoStart() {
        if (autoStartingPosition != null)
            return autoStartingPosition.getSelected();
        else
            return AutoStartPosition.Left;
    }

    public void setTestInstructions(String string) {
        if (testInstructions != null)
            testInstructions.setString(string);
    }

    public void setTestString(String string) {
        if (testString != null)
            testString.setString(string);
    }

    private void displayBoolean(GenericEntry entry, boolean value) {
        entry.setBoolean(value);
    }

    private void displayString(GenericEntry entry, String value) {
        entry.setString(value);
    }

    private void displayInteger(GenericEntry entry, int value) {
        entry.setInteger(value);
    }

    @SuppressWarnings("unused")
    private void displayDoubleRound0(GenericEntry entry, double value) {
        entry.setString(String.valueOf(RobotMath.round0(value)));
    }

    @SuppressWarnings("unused")
    private void displayDoubleRound1(GenericEntry entry, double value) {
        entry.setString(String.valueOf(RobotMath.round1(value)));
    }

    private void displayDoubleRound2(GenericEntry entry, double value) {
        entry.setString(String.valueOf(RobotMath.round2(value)));
    }

    private void displayDoubleRound3(GenericEntry entry, double value) {
        entry.setString(String.valueOf(RobotMath.round3(value)));
    }

}