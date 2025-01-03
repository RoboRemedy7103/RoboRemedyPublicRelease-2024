// RobotState.java - Place to store info about the robot so they can be calculated once and used multiple times
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.PhotonVisionCamera.*;
import frc.robot.RobotEnums.*;

public class RobotState {
    private Electronics e;
    private RoboLog rLog;

    public double robotStartTime = 0;
    public double robotElapsedTime = 0;
    public double commandTime = 0;
    public double commandElapsedTime = 0;
    public double lastCommandTime = 0;

    public static final double MIN_PIVOT_ANGLE = 19.5;
    public static final double MAX_PIVOT_ANGLE = 73.0;
    public static final double PIVOT_AIM_ANGLE = 68.5 + 1;
    public static final double MAX_ROBOT_DECELERATION = 55; //Inches per second squared

    public double pivotAngle;
    public double pivotAngleRounded;
    public double gyroAngle;
    public double gyroAngleRounded;
    public double gyroTimeStamp;

    public boolean isSimulation = false;
    public boolean isGamePieceCameraAttached = false;
    public boolean isMainAprilTagCameraAttached = false;
    public boolean isAprilTagFound = false;
    public double aprilTagInchesInFrontOfTarget = 0;
    public double aprilTagInchesRightOfTarget = 0;
    public double commandedVelocity = 0;
    public double distanceMoved = 0;
    public double mainTargetFacingAngle = 0;
    public double mainTargetPitch;
    public double mainTargetTimeStamp;
    
    public boolean isRedAlliance = true;

    public Position robotPosition = new Position("robot position", 0, 0, 0);
    public Position deadReckoningPosition = new Position("dead reckoning position", 0, 0, 0);
    public LinearMapper previousXPositions = new LinearMapper(4);
    public LinearMapper previousYPositions = new LinearMapper(4);
    public LinearMapper previousGyroHeadings = new LinearMapper(4);

    public static final double APRILTAG_YAW_OFFSET = 0;

    public boolean isMainTargetFound;
    public boolean isAmpFound;
    public double mainTargetYaw;
    public double mainTargetDistance;
    public double distanceToAmp;

    public TargetInfo mainTargetResult = new TargetInfo();

    private boolean lastIsGamePieceCameraAttached = false;
    private boolean lastIsMainAprilTagCameraAttached = false;

    public boolean isRobotOk;
    public boolean isFLCancoderGood;
    public boolean isFRCancoderGood;
    public boolean isBLCancoderGood;
    public boolean isBRCancoderGood;
    public boolean isFLDriveGood;
    public boolean isFLTurnGood;
    public boolean isFRDriveGood;
    public boolean isFRTurnGood;
    public boolean isBLDriveGood;
    public boolean isBLTurnGood;
    public boolean isBRDriveGood;
    public boolean isBRTurnGood;
    public boolean doSwerveTurnEncodersMatch;
    public boolean isSampleMotorGood;
    public boolean isPivotMotorGood;
    public boolean isPivotEncoderGood;
    public double batteryVoltage;
    public boolean isBatteryGood = false;

    public AutoProgram autoSelection;
    public AutoStartPosition autoStartPosition;

    private boolean lastFLCancoderGood = false;
    private boolean lastFRCancoderGood = false;
    private boolean lastBLCancoderGood = false;
    private boolean lastBRCancoderGood = false;
    private boolean lastFLDriveGood = false;
    private boolean lastFLTurnGood = false;
    private boolean lastFRDrivegood = false;
    private boolean lastFRTurnGood = false;
    private boolean lastBLDriveGood = false;
    private boolean lastBLTurnGood = false;
    private boolean lastBRDriveGood = false;
    private boolean lastBRTurnGood = false;
    private boolean lastSwerveTurnEncodersMatch = false;
    private boolean lastSampletMotorGood = false;
    private boolean lastPivotMotorGood = false;
    private boolean lastPivotEncoderGood = false;
    private double lastMainTargetTimeStamp;

    public AutoProgram lastAutoSelection = AutoProgram.None;
    public AutoStartPosition lastAutoStartPosition = AutoStartPosition.Left;

    public RobotState(Electronics e, RoboLog rLog) {
        this.e = e;
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: RobotState.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: RobotState.e is null");
        }

        isSimulation = RobotBase.isSimulation();
        robotStartTime = Timer.getFPGATimestamp();
    }

    public void robotStatePeriodic(Dashboard dash, SimulationRobot simulation) {
        lastCommandTime = commandTime;
        commandTime = Timer.getFPGATimestamp();
        commandElapsedTime = commandTime - lastCommandTime;
        robotElapsedTime = commandTime - robotStartTime;

        commandedVelocity = e.getLastTravelVelocityCommand();
        distanceMoved = commandedVelocity * commandElapsedTime;
 
        if (isSimulation == true) {
            simulation.beforeRobotStatePeriodic();
        }

        isGamePieceCameraAttached = e.getGamePieceCamera().isConnected();
        isMainAprilTagCameraAttached = e.getMainAprilTagCamera().isConnected();

        autoSelection = dash.geAutoProgram();
        autoStartPosition = dash.getAutoStart();

        gyroAngle = e.getGyroCenteredOnGoal(0);
        gyroAngleRounded = RobotMath.round1(gyroAngle);
        gyroTimeStamp = e.getGyroTimestamp();

        if (isSimulation) {
            batteryVoltage = simulation.getBatteryVoltage();
        } else {
            batteryVoltage = e.getBatteryVoltage();
        }

        if (batteryVoltage >= 12.3) {
            isBatteryGood = true;
        } else {
            isBatteryGood = false;
        }

        isRedAlliance = e.isAllianceRed();

        e.getMainAprilTagCamera().findSpecificAprilTag(1, mainTargetResult);
        isMainTargetFound = mainTargetResult.isFound;
        mainTargetPitch = mainTargetResult.pitch;
        mainTargetYaw = mainTargetResult.yaw + APRILTAG_YAW_OFFSET;
        lastMainTargetTimeStamp = mainTargetTimeStamp;
        mainTargetTimeStamp = mainTargetResult.timeStamp;

        if (!isMainTargetFound) {
            mainTargetFacingAngle = 0;
        } else if ((mainTargetTimeStamp != lastMainTargetTimeStamp)) {
            mainTargetFacingAngle = mainTargetYaw + gyroAngle;
        }

        pivotAngle = e.getPivotCancoderAngle();
        pivotAngleRounded = RobotMath.round1(pivotAngle);

        calcDeadReckoning();

        isSampleMotorGood = e.isSampletMotorAttached();
        isFLCancoderGood = e.isSwerveEncoderAttached(0);
        isFRCancoderGood = e.isSwerveEncoderAttached(1);
        isBLCancoderGood = e.isSwerveEncoderAttached(2);
        isBRCancoderGood = e.isSwerveEncoderAttached(3);
        isFLDriveGood = e.isSwerveDriveMotorAttached(0);
        isFLTurnGood = e.isSwerveTurnMotorAttached(0);
        isFRDriveGood = e.isSwerveDriveMotorAttached(1);
        isFRTurnGood = e.isSwerveTurnMotorAttached(1);
        isBRDriveGood = e.isSwerveDriveMotorAttached(2);
        isBRTurnGood = e.isSwerveTurnMotorAttached(2);
        isBLDriveGood = e.isSwerveDriveMotorAttached(3);
        isBLTurnGood = e.isSwerveTurnMotorAttached(3);
        //doSwerveTurnEncodersMatch = e.doSwerveEncodersMatch();
        isPivotMotorGood = e.isPivotMotorAttached();
        isPivotEncoderGood = e.isPivotCancoderAttached();

        isRobotOk = isMainAprilTagCameraAttached && isGamePieceCameraAttached
            && isSampleMotorGood && isPivotMotorGood
            && isFLDriveGood && isFLTurnGood
            && isFRDriveGood && isFRTurnGood
            && isBLDriveGood && isBLTurnGood
            && isBRDriveGood && isBRTurnGood && isFLCancoderGood && isFRCancoderGood
            && isBLCancoderGood && isBRCancoderGood
            && isPivotEncoderGood && isBatteryGood;

        if (robotElapsedTime > 3.0) {
            logChanges(); // Start logging changes after 3 seconds
        }
    }

    public void logChanges() {
        if (lastIsGamePieceCameraAttached != isGamePieceCameraAttached) {
            rLog.print("Game Piece Camera Attached: " + isGamePieceCameraAttached);
            lastIsGamePieceCameraAttached = isGamePieceCameraAttached;
        }
        if (lastIsMainAprilTagCameraAttached != isMainAprilTagCameraAttached) {
            rLog.print("April Tag Camera Attached: " + isMainAprilTagCameraAttached);
            lastIsMainAprilTagCameraAttached = isMainAprilTagCameraAttached;
        }
        if (lastFLDriveGood != isFLDriveGood) {
            rLog.print("Front Left Drive Good: " + isFLDriveGood);
            lastFLDriveGood = isFLDriveGood;
        }
        if (lastFLTurnGood != isFLTurnGood) {
            rLog.print("Front Left Turn Good: " + isFLTurnGood);
            lastFLTurnGood = isFLTurnGood;
        }
        if (lastFRDrivegood != isFRDriveGood) {
            rLog.print("Front Right Drive Good: " + isFRDriveGood);
            lastFRDrivegood = isFRDriveGood;
        }
        if (lastFRTurnGood != isFRTurnGood) {
            rLog.print("Front Right Turn Good: " + isFRTurnGood);
            lastFRTurnGood = isFRTurnGood;
        }
        if (lastBLDriveGood != isBLDriveGood) {
            rLog.print("Back Left Drive Good: " + isBLDriveGood);
            lastBLDriveGood = isBLDriveGood;
        }
        if (lastBLTurnGood != isBLTurnGood) {
            rLog.print("Back Left Turn Good: " + isBLTurnGood);
            lastBLTurnGood = isBLTurnGood;
        }
        if (lastBRDriveGood != isBRDriveGood) {
            rLog.print("Back Right Drive Good: " + isBRDriveGood);
            lastBRDriveGood = isBRDriveGood;
        }
        if (lastBRTurnGood != isBRTurnGood) {
            rLog.print("Back Right Turn Good : " + isBRTurnGood);
            lastBRTurnGood = isBRTurnGood;
        }
        if (lastSwerveTurnEncodersMatch != doSwerveTurnEncodersMatch) {
            rLog.print("Swerve Encoders Match: " + doSwerveTurnEncodersMatch);
            lastSwerveTurnEncodersMatch = doSwerveTurnEncodersMatch;
            }
        if (lastFLCancoderGood != isFLCancoderGood) {
            rLog.print("Front Left Cancoder Good: " + isFLCancoderGood);
            lastFLCancoderGood = isFLCancoderGood;
        }
        if (lastFRCancoderGood != isFRCancoderGood) {
            rLog.print("Front Right Cancoder Good: " + isFRCancoderGood);
            lastFRCancoderGood = isFRCancoderGood;
        }
        if (lastBLCancoderGood != isBLCancoderGood) {
            rLog.print("Back Left Cancoder Good: " + isBLCancoderGood);
            lastBLCancoderGood = isBLCancoderGood;
        }
        if (lastBRCancoderGood != isBRCancoderGood) {
            rLog.print("Back Right Cancoder Good: " + isBRCancoderGood);
            lastBRCancoderGood = isBRCancoderGood;
        }
        if (lastSampletMotorGood != isSampleMotorGood) {
            rLog.print("Sample Motor Good: " + isSampleMotorGood);
            lastSampletMotorGood = isSampleMotorGood;
        }
        if (lastPivotMotorGood != isPivotMotorGood) {
            rLog.print("Pivot Motor Good: " + isPivotMotorGood);
            lastPivotMotorGood = isPivotMotorGood;
        }
        if (lastPivotEncoderGood != isPivotEncoderGood) {
            rLog.print("Pivot Cancoder Good: " + isPivotEncoderGood);
            lastPivotEncoderGood = isPivotEncoderGood;
        }
        if (lastAutoSelection != autoSelection) {
            rLog.print("Auto Selection: " + autoSelection);
            lastAutoSelection = autoSelection;
            }
        if (lastAutoStartPosition != autoStartPosition) {
            rLog.print("Auto Start: " + autoStartPosition);
            lastAutoStartPosition = autoStartPosition;
        }
    }

    public void calcDeadReckoning() {
        double yDistance = Math.cos(Math.toRadians(e.getLastTravelAngleDegrees())) * distanceMoved;
        double xDistance = Math.sin(Math.toRadians(e.getLastTravelAngleDegrees())) * distanceMoved;
        deadReckoningPosition.addToPosition(xDistance, yDistance);
        deadReckoningPosition.setYaw(gyroAngle);

        // Store previous robot dead recokoning positions and previous gyro values
        previousXPositions.add(commandTime, deadReckoningPosition.getX());
        previousYPositions.add(commandTime, deadReckoningPosition.getY());
        previousGyroHeadings.add(commandTime, gyroAngle);

        if (isMainTargetFound && mainTargetTimeStamp != lastMainTargetTimeStamp) {
            // New Image found. Calculate position using vision, then add
            // change in dead reckoning since that time stamp
            double s = mainTargetDistance;
            double r = mainTargetYaw;
            double yaw = previousGyroHeadings.calculate(mainTargetTimeStamp);

            double targetAngle = r + yaw;

            double xFromTarget = s * Math.sin(Math.toRadians(targetAngle));
            double yFromTeaker = s * Math.cos(Math.toRadians(targetAngle));

            double x = previousXPositions.calculate(mainTargetTimeStamp);
            double y = previousYPositions.calculate(mainTargetTimeStamp);

            double xDistanceChange = deadReckoningPosition.getX() - x;
            double yDistanceChange = deadReckoningPosition.getY() - y;
            robotPosition.setPosition(xFromTarget + xDistanceChange, yFromTeaker + yDistanceChange);
        } else {
            // No Image found. Calculate position using commanded movement
            robotPosition.addToPosition(xDistance, yDistance);
            robotPosition.setYaw(gyroAngle);
        }
    }
}