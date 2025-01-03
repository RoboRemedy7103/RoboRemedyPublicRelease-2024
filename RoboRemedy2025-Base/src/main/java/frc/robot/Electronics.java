// Electronics.java - Controls the motors, gyro, encoders, solenoids, etc.
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.*;
import edu.wpi.first.wpilibj.util.*;
import frc.robot.LEDCountdown.*;
import frc.robot.RobotMotor.*;
import frc.robot.RobotEnums.*;

import java.io.*;
import java.util.*;
import java.awt.geom.*;

public class Electronics {
    private static final double SAMPLE_RAMP_SPEED = 10;
    private static final double PIVOT_RAMP_SPEED = 10;

    private static LinearMapper swerveVelocityMapper = new LinearMapper();
    private RobotName robotName;

    private boolean allConfigsSet = false;
    private boolean[] configsSet = new boolean[14]; //Number of motors on bot
    private RobotMotor[] motorArray;

    private RoboLog rLog;

    private Timer batteryTimer = new Timer();
    private double batteryVoltage = 0;
    public double lastMotorInitTime = 0;

    private double pitchOffset = 0.0;
    private double rollOffset = 0.0;

    // Swerve
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;
    private SwerveModule backLeft;
    private SwerveModule[] swerveDrives;
    private SwerveState currentState;
    private SwerveState centerPivotState;

    // Motors and Cancoders
    private RobotMotor sampleMotor;
    private RobotMotor pivotMotor;
    private RobotCANcoder pivotCancoder;

    // Buttons
    private DigitalInput yellowButton;

    // LEDs
    public LEDString ledString = new LEDString(9, 300);
    private LEDCountdown timerStrip = new LEDCountdown();

    // Cameras
    private PhotonVisionCamera gamePieceCamera;
    private PhotonVisionCamera mainAprilTagCamera;
    private LidarCamera lidarCamera;

    // Others
    public RobotGyro gyro = new RobotGyro(); // public for the dashboard
    private PowerDistribution pdp;

    // Last Values
    private double lastTravelVelocity = 0;
    private double lastTravelAngleDegrees = 0;
    private double lastTravelRotationDegreesPerSecond = 0;
    private double lastSamplePercent = 0;
    private double lastPivotPercent = 0;
    private double lastPivotAngle = 0;
    private double lastAssignedPivotAngle = 0;

    Electronics(boolean fullSwerve, RoboLog rLog) {
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: Electronics.rLog is null");
        }

        File jr = new File("/home/lvuser/jr.txt");
        File botchendo = new File("/home/lvuser/botchendo.txt");

        if (jr.isFile()) {
            rLog.print("Robot = Jerry Junior");
            robotName = RobotName.JerryJr;
        } else if (botchendo.isFile()) {
            rLog.print("Robot = Scorpio");
            robotName = RobotName.Botchendo;
        } else {
            rLog.print("Robot = Live 2025");
            robotName = RobotName.LIVE2025;
        }

        if (robotName == RobotName.JerryJr) {
            centerPivotState = new SwerveState(13.5, 13.5);

            currentState = centerPivotState;

            frontLeft = new SwerveModule(41, 51, 1, 0, rLog, 
                    RobotMotorType.TalonFX, RobotMotorType.SparkMax, RobotEncoderType.Cancoder, 
                    3.77, 8.308, 20);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax, 
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.SparkMax,
                    RobotEncoderType.Cancoder, 3.77, 8.308, 20);

            pdp = new PowerDistribution();
            yellowButton = null;

            sampleMotor = null;
            pivotMotor = null;
            pivotCancoder = null;
        } else if (robotName == RobotName.Botchendo) {
            centerPivotState = new SwerveState(new Point2D.Double[] {
                new Point2D.Double(-10.875,6),
                new Point2D.Double(10.875,6),
                new Point2D.Double(10.875,-10.25),
                new Point2D.Double(-10.875,-10.25),
            }
            );

            currentState = centerPivotState;

            swerveVelocityMapper.add(0.8, 0.027);
            swerveVelocityMapper.add(3.7, 0.035);
            swerveVelocityMapper.add(13.6, 0.067);
            swerveVelocityMapper.add(20.8, 0.1);
            swerveVelocityMapper.add(25.2, 0.119);
            swerveVelocityMapper.add(62.6, 0.3);
            swerveVelocityMapper.add(83.8, 0.399);
            swerveVelocityMapper.add(93.9, 0.45);
            swerveVelocityMapper.add(167.9, 0.8);
            swerveVelocityMapper.add(187.7, 0.9);
            swerveVelocityMapper.add(195.9, 0.94);

            // Swerves
            frontLeft = new SwerveModule(41, 51, 1, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);

            frontLeft.setVelocityMapper(swerveVelocityMapper);
            frontRight.setVelocityMapper(swerveVelocityMapper);
            backRight.setVelocityMapper(swerveVelocityMapper);
            backLeft.setVelocityMapper(swerveVelocityMapper);
            
            pdp = new PowerDistribution();
            pdp.setSwitchableChannel(true);
            yellowButton = new DigitalInput(3);

            sampleMotor = new RobotMotor(RobotMotor.RobotMotorType.TalonFX, 9, true, false, rLog, //isCoast was true
                /* 50:1 ratio, 1.5 inch diameter */ (1.5 * Math.PI / 16.0),
                0, 0, RobotEncoderType.Internal, 0, false);
            pivotMotor = new RobotMotor(RobotMotor.RobotMotorType.TalonFX, 8, false, false, rLog, 
                /* 300:1 ratio, 360 degrees per revolution */ (360.0 / 300.0),
                11, 40, RobotEncoderType.Internal, 0, false);//isCoast was true
            pivotCancoder = new RobotCANcoder(6, true, rLog);

            sampleMotor.setPID(1.5, 0.0000004, 0.01, 30, true);
        } else {
            centerPivotState = new SwerveState(new Point2D.Double[] {
                new Point2D.Double(-10.875,6),
                new Point2D.Double(10.875,6),
                new Point2D.Double(10.875,-10.25),
                new Point2D.Double(-10.875,-10.25),
            }
            );

            currentState = centerPivotState;

            swerveVelocityMapper.add(0.8, 0.027);
            swerveVelocityMapper.add(3.7, 0.035);
            swerveVelocityMapper.add(13.6, 0.067);
            swerveVelocityMapper.add(20.8, 0.1);
            swerveVelocityMapper.add(25.2, 0.119);
            swerveVelocityMapper.add(62.6, 0.3);
            swerveVelocityMapper.add(83.8, 0.399);
            swerveVelocityMapper.add(93.9, 0.45);
            swerveVelocityMapper.add(167.9, 0.8);
            swerveVelocityMapper.add(187.7, 0.9);
            swerveVelocityMapper.add(195.9, 0.94);

            // Swerves
            frontLeft = new SwerveModule(41, 51, 1, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            frontRight = new SwerveModule(42, 52, 2, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backRight = new SwerveModule(43, 53, 3, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);
            backLeft = new SwerveModule(44, 54, 4, 0, rLog, RobotMotorType.TalonFX, RobotMotorType.TalonFX,
                    RobotEncoderType.Cancoder, 3.91, 6.12, 28.14);

            frontLeft.setVelocityMapper(swerveVelocityMapper);
            frontRight.setVelocityMapper(swerveVelocityMapper);
            backRight.setVelocityMapper(swerveVelocityMapper);
            backLeft.setVelocityMapper(swerveVelocityMapper);
            
            pdp = new PowerDistribution();
            pdp.setSwitchableChannel(true);
            yellowButton = new DigitalInput(3);

            sampleMotor = new RobotMotor(RobotMotor.RobotMotorType.TalonFX, 9, true, false, rLog, //isCoast was true
                /* 50:1 ratio, 1.5 inch diameter */ (1.5 * Math.PI / 16.0),
                0, 0, RobotEncoderType.Internal, 0, false);
            pivotMotor = new RobotMotor(RobotMotor.RobotMotorType.TalonFX, 8, false, false, rLog, 
                /* 300:1 ratio, 360 degrees per revolution */ (360.0 / 300.0),
                11, 40, RobotEncoderType.Internal, 0, false);//isCoast was true
            pivotCancoder = new RobotCANcoder(6, true, rLog);

            sampleMotor.setPID(1.5, 0.0000004, 0.01, 30, true);
        }

        lidarCamera = new LidarCamera("lidar");

        lidarCamera.setRegion(0, 159, 30);

        if (fullSwerve) {
            swerveDrives = new SwerveModule[] { frontLeft, frontRight, backRight, backLeft };
        } else {
            swerveDrives = new SwerveModule[] { backLeft };
        }

        gamePieceCamera = new PhotonVisionCamera("Arducam_OV9782_USB_Camera (1)", rLog);
        mainAprilTagCamera = new PhotonVisionCamera("Arducam_OV9281_USB_Camera (1)", rLog);

        timerStrip.add(248, 1);
        timerStrip.add(249, 2);
        timerStrip.add(250, 3);
        timerStrip.add(251, 4);
        timerStrip.add(252, 5);
        timerStrip.add(253,6);
        timerStrip.add(254, 7);
        timerStrip.add(255,8);
        timerStrip.add(256, 9);
        timerStrip.add(257, 10);
        timerStrip.add(258, 11);
        timerStrip.add(259, 12);
        timerStrip.add(260, 13);
        timerStrip.add(261, 14);
        timerStrip.add(262, 15);
        timerStrip.add(263, 16);
        timerStrip.add(264, 17);
        timerStrip.add(265, 18);
        timerStrip.add(266, 19);
        timerStrip.add(267, 20);
        timerStrip.add(268, 21);
        timerStrip.add(269, 22);
        timerStrip.add(270, 23);
        timerStrip.add(271, 24);
        timerStrip.add(272, 25);
        timerStrip.add(273, 26);
        timerStrip.add(274, 27);
        timerStrip.add(275, 28);
        timerStrip.add(276, 29);
        timerStrip.add(277, 30);

        if (pdp != null) {
            batteryVoltage = pdp.getVoltage();
        }

        batteryTimer.restart();

        motorArray = new RobotMotor[] {
            sampleMotor,
            pivotMotor,
            frontLeft.getDriveMotor(),
            frontLeft.getTurnMotor(),
            frontRight.getDriveMotor(),
            frontRight.getTurnMotor(),
            backLeft.getDriveMotor(),
            backLeft.getTurnMotor(),
            backRight.getDriveMotor(),
            backRight.getTurnMotor(),
        };
    }

    public double getCurrent(int channel) {
        return pdp != null ? pdp.getCurrent(channel) : 0;
    }

    public boolean doSwerveEncodersMatch() {
        boolean encodersMatch = true;
        for (SwerveModule swerveModule : swerveDrives) {
            if (Math.abs(swerveModule.getTurnEncoderPosition() - swerveModule.getAbsoluteTurnEncoderPosition()) > 3.0) {
                encodersMatch = false;//Constant in if statement is degrees of tolerance
            }
        }
        return encodersMatch;
    }

    public void checkConfigsSet(boolean forcingConfig) {
        int loopCounter = 0;
        boolean isSet = true;
        for (RobotMotor motor : motorArray) {
            if (motor == null) {//Assumes null motors aren't meant to exist
                configsSet[loopCounter] = true;
            } else if (!motor.isAttached()) {
                //Do nothing (might break stuff)
            } else if (motor.isFXConfigNeeded() || forcingConfig) {
                if (!motor.configReapply()) {
                    rLog.print("Reapplying config failed on ID" + motorArray[loopCounter].getMotorID() + 
                            ", trying again");
                    configsSet[loopCounter] = false;
                } else {
                    configsSet[loopCounter] = true;
                    rLog.print("Reapplying config worked on " + motorArray[loopCounter].getMotorID());
                }
            } else {
                configsSet[loopCounter] = true;//Assumes other motor types did it successfully
            }
            loopCounter += 1;
        }
        for (boolean motorSet : configsSet) {//If any motor didn't set properly, isSet is false
            if (motorSet == false)
                isSet = false;
        }
        allConfigsSet = isSet;
    }

    public boolean areAllConfigsSet() {
        return allConfigsSet;
    }

    public void stopSwerveMotors() {
        lastTravelVelocity = 0;
        lastTravelRotationDegreesPerSecond = 0;
        for (SwerveModule m : swerveDrives) {
            m.setDriveSpeed(0);
            m.setTurnPower(0);
        }
    }

    public void alignSwerveMotorsForward() {
        lastTravelVelocity = 0;
        lastTravelAngleDegrees = 0;
        lastTravelRotationDegreesPerSecond = 0;
        for (SwerveModule m : swerveDrives) {
            m.setDrivePercent(0);
            m.setTurnHeading(0);
        }
    }

    public void setSampleMotorPercent(double motorPower) {
        if (sampleMotor != null) {
            sampleMotor.rampToPercent(motorPower, SAMPLE_RAMP_SPEED);
        }
        if (lastSamplePercent != motorPower) {
            rLog.print("Sample Power:" + motorPower);
            lastSamplePercent = motorPower;
        }
    }

    public void stopSampleMotor() {
        setSampleMotorPercent(0);
    }

    public void setPivotMotorPercent(double pivotPercent) {
        if (pivotMotor != null) {
            pivotMotor.rampToPercent(pivotPercent, PIVOT_RAMP_SPEED);
        }
        if (lastPivotPercent != pivotPercent) {
            lastPivotPercent = pivotPercent;
        }
    }   

    public void stopPivotMotor() {
        if (pivotMotor != null) {
            pivotMotor.rampToPercent(0, PIVOT_RAMP_SPEED);
            lastPivotPercent = 0;
        }
    }

    /**
     * Move pivot motor to a specific angle. Uses default acceptable range.
     * 
     * @param desiredAngle    Target angle, in degrees
     */
    public void movePivotToAngle(double desiredAngle) {
        movePivotToAngle(desiredAngle, .25);
    }

    /**
     * Move pivot motor to a specific angle
     * 
     * @param desiredAngle    Target angle, in degrees
     * @param acceptableRange How precise should the final angle be, in degrees
     */
    public void movePivotToAngle(double desiredAngle, double acceptableRange) {
        double actualAngle = getPivotCancoderAngle();
        double difference = desiredAngle - actualAngle;
        double speed = 0;
        if (difference > 0) {
            speed = (difference * 0.035) + 0.02;
        } else {
            speed = (difference * 0.015) - 0.015;
        }
        speed = RobotMath.minMax(-0.5, 0.5, speed);
        if (Math.abs(difference) < acceptableRange) {
            speed = 0;
        }
        setPivotMotorPercent(speed);
        if (lastPivotAngle != actualAngle) {
            lastPivotAngle = actualAngle;
        }
    }

    /***
     * Set the absolute encoder angle. Used by the "Reset Pivot Encoder" test.
     * 
     * @param angle Desired absolute angle, in degrees
     */
    public void setPivotEncoderAngle(double angle) {
        if (pivotMotor != null)
            pivotCancoder.setAbsoluteAngle(angle);
    }

    /**
     * Gets the current reading of the canCoder.
     * 
     * @return the current angle, in degrees
     */
    public double getPivotCancoderAngle() {
        if (pivotCancoder != null) {
            return pivotCancoder.getAbsoluteAngle();
        }
        else
            return 0;
    }

    public double getPivotVelocity() {
        if (pivotMotor != null) {
            return pivotMotor.getEncoderVelocity();
        } else {
            return 0;
        }
    }

    public void stopAllMotors() { // Stops other motors, too
        stopSwerveMotors();
        stopSampleMotor();
        stopPivotMotor();
    }

    private SwerveModule getModule(int module) {
        if (module >= swerveDrives.length || module < 0) {
            return null;
        } else {
            return swerveDrives[module];
        }
    }

    public void setDrivePercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDrivePercent(power);
        }
    }

    public void setDriveSpeed(int moduleNumber, double velocity) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Module ");
        } else {
            swerveDriveModule.setDriveSpeed(velocity);
        }
    }

    public RobotMotor getSwerveDriveMotor(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return null;
        } else {
            return swerveDriveModule.getDriveMotor();
        }
    }

    public void setTurnPercent(int moduleNumber, double power) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnPower(power);
        }
    }

    public void setTurnHeading(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnHeading(angle);
        }
    }

    public void setAllHeadings(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnHeading(i, angle);
        }
    }

    public void setTurnEncoder(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setTurnEncoderValue(angle);
        }
    }

    /***
     * Sets the turn motor internal encoders to match the absolute encoder values
     * for all swerve modules.
     */
    public void assignAllSwerveEncoderValues() {
        for (int i = 0; i < 4; i++) {
            setTurnEncoder(i, getAbsoluteTurnEncoderPosition(i));
        }
        rLog.print("Swerve Drive Turn Encoders Reset");
    }

    public boolean initSwerveTurnEncoders() {
        for (int i = 0; i < 4; i++) {
            swerveDrives[i].setTurnEncoderSet(swerveDrives[i].setSwerveTurnEncoderValue());
        }
        if (swerveDrives[0].isTurnEncoderSet() && swerveDrives[1].isTurnEncoderSet() && swerveDrives[2].isTurnEncoderSet()
                && swerveDrives[3].isTurnEncoderSet()) {
            return true;
        }
        return false;
    }

    public void setAbsoluteEncoderValue(int moduleNumber, double angle) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
        } else {
            swerveDriveModule.setAbsoluteTurnEncoderPosition(angle);
        }
    }

    public void zeroAllAbsoluteEncoderValues() {
        for (int i = 0; i < 4; i++) {
            setAbsoluteEncoderValue(i, 0);
        }
    }

    public void setAllTurnEncoders(double angle) {
        for (int i = 0; i < swerveDrives.length; i++) {
            setTurnEncoder(i, angle);
        }
    }

    public double[] getAllTurnEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getTurnEncoderPosition();
        }
        return positions;
    }

    public double[] getAllDriveEncoders() {
        double[] positions = new double[swerveDrives.length];
        for (int i = 0; i < swerveDrives.length; i++) {
            positions[i] = swerveDrives[i].getDriveEncoderPosition();
        }
        return positions;
    }

    public double getDriveVelocity(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveVelocity();
        }
    }

    public double getDriveOutputPercentage(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getDriveOutputPercentage();
        }
    }

    public boolean areWheelsStopped() {
        for (SwerveModule m : swerveDrives) {
            if (Math.abs(m.getDriveVelocity()) > 0.5) {
                return false;
            }
        }
        return true;
    }

    public double getAbsoluteTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getAbsoluteTurnEncoderPosition();
        }
    }

    public double getTurnEncoderPosition(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            rLog.print("Not Valid Wheel");
            return 0;
        } else {
            return swerveDriveModule.getTurnEncoderPosition();
        }
    }

    public void assignRobotMotionField(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastTravelAngleDegrees = travelAngle;
        lastTravelRotationDegreesPerSecond = degreesPerSecond;
        currentState.assignSwerveModulesField(travelAngle, travelInchesPerSecond, degreesPerSecond, getGyro(),
                swerveDrives[0].getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(currentState.getMagnitude(i), currentState.getAngle(i));
        }
    }

    /** Gets the last timestamp reported by the gyro. */
    public double getGyroTimestamp() {
        if (gyro != null)
            return gyro.getLastSensorTimestamp() / 1000.0;
        else
            return 0;
    }

    public void assignRobotMotionRobot(double travelAngle, double travelInchesPerSecond, double degreesPerSecond) {
        lastTravelVelocity = travelInchesPerSecond;
        lastTravelAngleDegrees = travelAngle + getGyro();
        lastTravelRotationDegreesPerSecond = degreesPerSecond;
        currentState.assignSwerveModules(travelAngle, travelInchesPerSecond, degreesPerSecond,
                swerveDrives[0].getMaxVelocity());
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setSpeedAndHeading(currentState.getMagnitude(i), currentState.getAngle(i));
        }
    }

    public void assignRobotMotionAndHeadingField(double travelAngle, double travelInchesPerSecond, double facingAngle) {
        double degreesPerSecond = 0;
        double angleDifference = facingAngle - getGyroCenteredOnGoal(facingAngle);
        double acceptableRange = Math.max(Math.min((0.1042 * travelInchesPerSecond), 0.6), 0.6);
        if (Math.abs(angleDifference) < acceptableRange) {
            degreesPerSecond = 0;
        } else {
            double calcDPS = angleDifference * 4.0;
            double rotationSpeed = 180;
             if (calcDPS < 0) {
                degreesPerSecond = Math.min(Math.max(-rotationSpeed, calcDPS), -3.5);
            } else {
                degreesPerSecond = Math.max(Math.min(rotationSpeed, calcDPS), 3.5);
            }
        }
        assignRobotMotionField(travelAngle, travelInchesPerSecond, degreesPerSecond);
    }

    public void lockWheels() {
        lastTravelVelocity = 0;
        lastTravelRotationDegreesPerSecond = 0;
        currentState.lockWheels();
        for (int i = 0; i < swerveDrives.length; i++) {
            swerveDrives[i].setDrivePercent(currentState.getMagnitude(i));
            swerveDrives[i].setTurnHeading(currentState.getAngle(i));
        }
    }

    /**
     * Is one of the swerve encoders connected?
     * 
     * @param moduleNumber Which swerve module? 0 is first module.
     */
    public boolean isSwerveEncoderAttached(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            return false;
        } else {
            return swerveDriveModule.isEncoderAttached();
        }
    }

    /**
     * Is one of the swerve turn motors connected?
     * 
     * @param moduleNumber Which swerve module? 0 is first module.
     */
    public boolean isSwerveTurnMotorAttached(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            return false;
        } else {
            return swerveDriveModule.isTurnAttached();
        }
    }

    /**
     * Is one of the swerve drive motors connected?
     * 
     * @param moduleNumber Which swerve module? 0 is first module.
     */
    public boolean isSwerveDriveMotorAttached(int moduleNumber) {
        SwerveModule swerveDriveModule = getModule(moduleNumber);
        if (swerveDriveModule == null) {
            return false;
        } else {
            return swerveDriveModule.isDriveAttached();
        }
    }

    public boolean isPivotCancoderAttached() {
        if (pivotCancoder == null)
            return false;
        else {
            return pivotCancoder.isAttached();
        }
    }

    public double getLastTravelVelocityCommand() {
        return lastTravelVelocity;
    }

    public double getLastTravelAngleDegrees() {
        return lastTravelAngleDegrees;
    }

    public double getLastTravelRotationDegreesPerSecond() {
        return lastTravelRotationDegreesPerSecond;
    }

    public double getLastSampleMotorPercent() {
        return lastSamplePercent;
    }

    public double getLastPivotMotorPercent() {
        return lastPivotPercent;
    }

    public double getLastAssignedPivotAngle() {
        return lastAssignedPivotAngle;
    }

    public void setSwerveCenterPoint(double x, double y) {
        currentState = new SwerveState(centerPivotState, x, y);
    }

    public void setSwerveCoast() {
        for (SwerveModule m : swerveDrives) {
            m.setDriveAndTurnNeutralMode(true);
        }
    }

    public void setSwerveBrake() {
        for (SwerveModule m : swerveDrives) {
            m.setDriveAndTurnNeutralMode(false);
        }
    }

    public double getGyro() {
        if (gyro != null)
            return gyro.getYaw();
        else
            return 0.0;
    }

    public double getGyroCenteredOnGoal(double goalAngle) {
        double gyroValue = getGyro();

        return RobotMath.angleCenteredOnTarget(gyroValue, goalAngle);
    }

    /**
     * Tells the gyro which direction the robot is currently facing
     * 
     * @param gyroDegrees Angle in degrees
     */
    public void setGyro(double gyroDegrees) {
        if (gyro != null)
            gyro.setYaw(gyroDegrees);
        rLog.print("Gyro set: " + gyroDegrees);
    }

    /**
     * Called by the simulator to set the robot's current facing angle
     * 
     * @param gyroDegrees Angle in degrees
     */
    public void setSimulationGyro(double gyroDegrees) {
        if (gyro != null) {
            gyro.setSimulationYaw(gyroDegrees);
        }
    }

    public boolean getYellowButtonValue() {
        if (yellowButton == null) {
            return false;
        } else {
            return yellowButton.get();
        }
    }

    public String getRobotName() {
        String name;
        if (robotName == RobotName.JerryJr) {
            name = "Jerry Jr";
        } else if (robotName == RobotName.Botchendo) {
            name = "Scorpio";
        } else {
            name = "Live 2024";
        }
        return name;
    }

    public boolean isAllianceRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return (alliance.isPresent() && alliance.get() == Alliance.Red);
    }

    public void robotPeriodic() {
        if (batteryTimer.get() >= 10 && pdp != null) {
            batteryVoltage = pdp.getVoltage();
            batteryTimer.restart();
        }
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getTotalAmps() {
        return pdp.getTotalCurrent();
    }

    public void setLED(LEDregion region, LEDcolor color) {
        int start = 0;
        int end = 0;
        if (region == LEDregion.RobotStatus) {
            start = 0;
            end = 9;
        } else if (region == LEDregion.BellyPan) {
            start = 10;
            end = 19;
        } else if (region == LEDregion.Alliance) {
            start = 20;
            end = 29;
        } else if (region == LEDregion.AutoStep) {
            start = 30;
            end = 39;
        } else if (region == LEDregion.RegionAll) {
            start = 0;
            end = 39;
        } else {
            start = 0;
            end = 0;
        }

        ledString.setLEDs(start, end, color);
    }

    public void setLEDsForCountdown(double timeRemaining) {
        Color color1 = new Color(10, 20, 30);
        Color color2 = new Color(30, 10, 10);
        for (SingleLED led : timerStrip.LEDcount) {
            //rLog.print("LED:" + led.number + "," + led.time + "," + timeRemaining);
            if (timeRemaining < led.time) {
                ledString.setLED(led.number, color1);
            } else {
                ledString.setLED(led.number, color2);
            }
        }
    }

    public void sendLEDBuffer() {
        ledString.sendLEDBuffer();
    }

    /**
     * Use gyro to determine the roll angle of the robot
     * 
     * @return Robot's roll, in degrees
     */
    public double getRobotRoll() {
        if (gyro != null)
            return gyro.getRoll() - rollOffset;
        else
            return 0;
    }

    /**
     * Use gyro to determine the pitch angle of the robot
     * 
     * @return Robot's pitch, in degrees
     */
    public double getRobotPitch() {
        if (gyro != null)
            return gyro.getPitch() - pitchOffset;
        else
            return 0;
    }

    public void resetTilt() {
        if (gyro == null)
            return;
        else {
            pitchOffset = gyro.getPitch();
            rollOffset = gyro.getRoll();
        }
    }

    public boolean isSampletMotorAttached() {
        if (sampleMotor == null)
            return false;
        else {
            return sampleMotor.isAttached();
        }
    }

    public boolean isPivotMotorAttached() {
        if (pivotMotor == null)
            return false;
        else {
            return pivotMotor.isAttached();
        }
    }
    
    public PhotonVisionCamera getGamePieceCamera() {
        return gamePieceCamera;
    }

    public PhotonVisionCamera getMainAprilTagCamera() {
        return mainAprilTagCamera;
    }

    public boolean isLidarCameraConnected() {
        if (lidarCamera == null)
            return false;
        else
            return lidarCamera.isConnected();
    }

    public double getLidarDistance() {
        if (lidarCamera == null)
            return -3;
        else
            return lidarCamera.getRegionZ();
    }
}
