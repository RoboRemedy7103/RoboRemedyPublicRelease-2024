// Action.java - Action that can be used for autonomous behaviour
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class Action {
    private Electronics e;
    private RoboLog rLog;
    private RobotState robotState;
    private OI oi;

    private static final double ROBOT_MIN_SPEED = 5;

    private double[] driveEncoderZeros;
    private Timer actionTimer = new Timer();
    private Timer logTimer = new Timer();
    private double calcDistTraveled = 0;
    private double extraDriveDistance = 0;
    private boolean lastDriverStationAttached = false;
    private boolean isPrinted = false;
    private static final double DRIVE_STRAIGHT_TARGET_RANGE = 0.5;

    Action(Electronics elec, RoboLog rLog, OI oi, RobotState robotState) {
        this.rLog = rLog;
        this.e = elec;
        this.oi = oi;
        this.robotState = robotState;

        if (this.rLog == null) {
            System.out.println("Warning: Action.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: Action.e is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: Action.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: Action.robotState is null");
        }

        actionTimer.restart();
        logTimer.restart();
    }

    /**
     * Resets all variables used in actions.
     */
    public void resetAction() {
        driveEncoderZeros = e.getAllDriveEncoders();
        calcDistTraveled = 0;
        extraDriveDistance = 0;
        actionTimer.restart();
        isPrinted = false;
    }

    /**
     * Gets the action timer's value and returns it as a double.
     * 
     * @return The current time on actionTimer
     */
    public double getActTime() {
        return actionTimer.get();
    }

    /**
     * Drive straight a certain distance without attempting to keep the robot facing
     * a specific direction.
     * The distance traveled will be more accurate than when using
     * driveStraightWithFacing
     * 
     * @param travelAngle           Which way the bot will move
     * @param travelInchesPerSecond How fast the robot will move
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     */
    public boolean driveStraightNoFacing(double travelAngle, double travelInchesPerSecond, double maxTravelAcceleration,
            double distance, double endingInchesPerSecond) {
        double distTraveled = Math.abs((e.getAllDriveEncoders()[0] - driveEncoderZeros[0]));
        double distYetToGo = distance - distTraveled;
        if (distTraveled > distance - DRIVE_STRAIGHT_TARGET_RANGE) {
            e.assignRobotMotionField(travelAngle, endingInchesPerSecond, 0);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double calcInPerSec = calcInchesPerSec(travelInchesPerSecond, distYetToGo, maxTravelAcceleration, endingInchesPerSecond);
            e.assignRobotMotionField(travelAngle, calcInPerSec, 0);
        }
        return false;
    }

    /**
     * Drive straight a certain distance while attempting to keep the robot facing a
     * specific direction.
     * The distance traveled will be less accurate than when using
     * driveStraightNoFacing.
     * 
     * @param travelAngle           Which way the bot will move
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           Which way the robot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     */
    public boolean driveStraightWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration, double distance, double endingInchesPerSecond) {
        calcDistTraveled += robotState.distanceMoved;
        double distYetToGo = distance - calcDistTraveled + extraDriveDistance;
        if (calcDistTraveled > distance + extraDriveDistance - DRIVE_STRAIGHT_TARGET_RANGE) {
            e.assignRobotMotionAndHeadingField(travelAngle, endingInchesPerSecond, facingAngle);
            if (extraDriveDistance != 0)
                rLog.print("DriveStraight Done. Distance: " + RobotMath.round1(distance) +
                    " Extra: " + RobotMath.round1(extraDriveDistance));
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double calcInPerSec = calcInchesPerSec(travelInchesPerSecond, distYetToGo, maxTravelAcceleration, endingInchesPerSecond);
            e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
        }
        return false;
    }

    /**
     * Drive straight a certain distance while attempting to keep the robot facing a
     * specific direction. Instead of distance and angle, it takes a distance forward
     * and a distance to the right.
     * 
     * @param inchesForward         How far toward the front to drive
     * @param inchesRight           How far to the right of the robot to drive
     * @param travelInchesPerSecond How fast the robot will drive
     * @param facingAngle           Which way the robot will face
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond What speed the robot will slow down to before
     *                              moving on
     */
    public boolean driveStraightToPointWithFacing(double inchesForward, double inchesRight,
            double travelInchesPerSecond, double facingAngle, double maxTravelAcceleration,
            double endingInchesPerSecond) {
        double travelAngle = Math.toDegrees(Math.atan2(inchesRight, inchesForward));
        double distance = RobotMath.pythagoreanTheorem(inchesForward, inchesRight);
        if (!isPrinted) {
            rLog.print("calc travelAngle: " + RobotMath.round1(travelAngle)
                + ", distance: " + RobotMath.round1(distance));
            isPrinted = true;
        }
        return driveStraightWithFacing(travelAngle, travelInchesPerSecond, facingAngle, maxTravelAcceleration,
                distance, endingInchesPerSecond);
    }

    private double calcInchesPerSec(double maxTravelSpeed, double distanceYetToGo,
            double maxTravelAcceleration, double endingInchesPerSecond) {
         // Limit the acceleration at the beginning
        double upInPerSec = limitAccel(maxTravelSpeed, maxTravelAcceleration);
        // Ramp down at the end based on distance to target. Adjusting the decelaration will affect how
        // fast or slow it ramps down. A larger value is more agressive. Reduce it if it overshoots.
        double deceleration = Math.min(maxTravelAcceleration, RobotState.MAX_ROBOT_DECELERATION);
        double dwnInPerSec = Math.sqrt((endingInchesPerSecond * endingInchesPerSecond) + (2 * deceleration * distanceYetToGo));
        // Also, don't go slower than ROBOT_MIN_SPEED. This contstant should be set to the maximum speed
        // that the robot can travel and still stop immediately.
        double calcInPerSec = Math.max(Math.min(upInPerSec, dwnInPerSec), ROBOT_MIN_SPEED);
        return calcInPerSec;
    }

    /**
     * Drive straight to a certain point on the field.
     * 
     * @param currentX              Robot's current X position
     * @param currentY              Robot's current Y position
     * @param desiredX              The target point's X position
     * @param desiredY              The target point's Y position
     * @param xRange                Tolerance in the X direction
     * @param yRange                Tolerance in the Y direction
     * @param facingAngle           Which way the robot will face
     * @param maxInchesPerSecond    How fast the robot will drive
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond Target speed at the end of action
     */
    public boolean driveToSpecifiedPoint(double currentX, double currentY, double desiredX, double desiredY, double xRange, double yRange, 
            double facingAngle, double maxInchesPerSecond, double maxTravelAcceleration, double endingInchesPerSecond) {
        double xMovement = desiredX - currentX;
        double yMovement = desiredY - currentY;
        double travelAngle = Math.toDegrees(Math.atan2(xMovement, yMovement));
        double distance = RobotMath.pythagoreanTheorem(xMovement, yMovement);

        double calcInPerSec = calcInchesPerSec(maxInchesPerSecond, distance, 
            maxTravelAcceleration, endingInchesPerSecond);

        if (Math.abs(xMovement) < xRange && Math.abs(yMovement) < yRange) {
            // Robot is within the correct ranges. Motion is done.
            if (endingInchesPerSecond == 0) {
                e.stopSwerveMotors();
            } else {
                e.assignRobotMotionAndHeadingField(travelAngle, endingInchesPerSecond, facingAngle);
            }
            return true;
        }
        e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
        return false;
    }

    /**
     * Drive a curve with a specific turn radius at a constant speed the whole way
     * while maintaining the current facing angle, whatever that happens to be.
     * Picture the
     * robot as driving part of an imaginary circle.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param maxInchesPerSecond    How fast the robot will drive
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     * @param turnRadius            The radius of the imaginary circle that the
     *                              robot drives on.
     */
    public boolean driveCurveRadiusNoFacing(double startingAngle, double targetAngle, double maxInchesPerSecond,
            double maxTravelAcceleration, double turnRadius) {
        return driveCurveRadiusWithFacing(startingAngle, targetAngle, maxInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, turnRadius);
    }

    /**
     * Drive a curve with a specific turn radius at a constant speed the whole way
     * while attempting to keep the robot facing a specific direction. Picture the
     * robot
     * as driving part of an imaginary circle.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           What direction the robot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     * @param turnRadius            The radius of the imaginary circle that the
     *                              robot drives on.
     */
    public boolean driveCurveRadiusWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double turnRadius) {
        double degreesPerSecond = (travelInchesPerSecond * 180) / (Math.PI * turnRadius);
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, facingAngle,
                maxTravelAcceleration, degreesPerSecond);
    }

    /**
     * Normally use driveCurveRadiusNoFacing instead.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param degreesPerSecond      How many degrees of the circle the robot would
     *                              cover per second at its current speed
     */
    public boolean driveCurveNoFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double maxTravelAcceleration, double degreesPerSecond) {
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, degreesPerSecond);
    }

    /**
     * Normally use driveCurveRadiusWithFacing instead.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           Which way the robot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param degreesPerSecond      How many degrees of the circle the robot would
     *                              cover at its current speed
     */
    public boolean driveCurveWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double degreesPerSecond) {
        boolean goalReached = false;
        if (startingAngle > targetAngle && degreesPerSecond > 0) {
            degreesPerSecond = -degreesPerSecond;
        }
        double goalHeading = startingAngle + (getActTime() * degreesPerSecond);
        if (degreesPerSecond > 0) {
            goalReached = targetAngle - goalHeading < 0.5;
        } else {
            goalReached = targetAngle - goalHeading > -0.5;
        }
        if (goalReached) {
            e.assignRobotMotionAndHeadingField(targetAngle, travelInchesPerSecond, facingAngle);
            return true;
        } else {
            double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            e.assignRobotMotionAndHeadingField(goalHeading, calcInPerSec, facingAngle);
        }
        return false;
    }

    /**
     * Drive straight at a specific speed with no specific ending point
     * 
     * @param travelAngle           Which way the bot will move
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           Which way the bot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     */
    public void driveStraightContinuousWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration) {
        double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
        e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
    }

    /**
     * Drive straight to a specific location using the PhotonVision data
     * 
     * @param facingAngle              Which way the bot will face while driving
     * @param targetDistanceInInches   How far in front of the target the bot wats
     *                                 to be
     * @param targetInchesRightOfTarget How far to the left the robot wants to be
     *                                 from the target
     * @param distanceRange            How precise the robot will try to be going
     *                                 forward/back
     * @param horzDistRange            How precise the robot will try to be going
     *                                 right/left
     * @param maxTravelSpeed           How fast the robot will move
     * @param maxTravelAcceleration    How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond    What the robot will slow down to before
     *                                 considering this action done
     */
    public boolean driveToPhotonVisionTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfTarget, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, double endingInchesPerSecond) {
        double forwardDistToTarget = robotState.aprilTagInchesInFrontOfTarget;
        double inchesRightOfTarget = robotState.aprilTagInchesRightOfTarget;
        boolean isAprilTagFound = robotState.isAprilTagFound;
        return driveToVisionTarget(facingAngle, targetDistanceInInches, targetInchesRightOfTarget, distanceRange,
                horzDistRange, maxTravelSpeed, maxTravelAcceleration, endingInchesPerSecond, isAprilTagFound,
                forwardDistToTarget, inchesRightOfTarget);
    }

    /**
     * Drive straight to a certain point on the field, using the robot state's robot position
     * 
     * @param desiredX              The target point's X position
     * @param desiredY              The target point's Y position
     * @param xRange                Tolerance in the X direction
     * @param yRange                Tolerance in the Y direction
     * @param facingAngle           Which way the robot will face
     * @param maxInchesPerSecond    How fast the robot will drive
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond Target speed at the end of action
     */
    public boolean driveToRobotStateTarget(double desiredX, double desiredY,
            double xRange, double yRange,
            double facingAngle, double maxInchesPerSecond, 
            double maxTravelAcceleration, double endingInchesPerSecond) {
        return driveToSpecifiedPoint(robotState.robotPosition.getX(), robotState.robotPosition.getY(),
            desiredX, desiredY, xRange, yRange, facingAngle, 
            maxInchesPerSecond, maxTravelAcceleration, endingInchesPerSecond);
    }

    /**
     * Drive to a specific location using any vision data. Goes forward or not at all.
     * 
     * @param facingAngle              Which way the bot will face while driving
     * @param targetDistanceInInches   How far in front of the target the bot wats
     *                                 to be
     * @param targetInchesRightOfTarget How far to the right the robot wants to be
     *                                 from the target
     * @param distanceRange            How precise the robot will try to be going
     *                                 forward/back
     * @param horzDistRange            How precise the robot will try to be going
     *                                 right/left
     * @param maxTravelSpeed           How fast the robot will move
     * @param maxTravelAcceleration    How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond    What the robot will slow down to before
     *                                 considering this action done
     * @param isTargetFound            Whether or not target is found, usually
     *                                 assigned by other methods
     * @param forwardDistToTarget      Current distance in fronnt of target, usually
     *                                 assigned by other methods
     * @param inchesRightOfTarget      Current distance to the right of target,
     *                                 usually assignned by other methods
     */
    public boolean driveToVisionTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfTarget, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, double endingInchesPerSecond, boolean isTargetFound,
            double forwardDistToTarget,
            double inchesRightOfTarget) {
        double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        if (isTargetFound && forwardDistToTarget > 0) {
            double horzDistToTarget = targetInchesRightOfTarget
                    - inchesRightOfTarget;
            double forwardDistToGoal = forwardDistToTarget - targetDistanceInInches;
            double calcDistToGo = Math.sqrt((Math.pow(horzDistToTarget, 2)) + Math.pow(forwardDistToGoal, 2));
            double calcHeading = gyroAngle + (90 - Math.toDegrees(Math.atan2(forwardDistToGoal, horzDistToTarget)));
            if (Math.abs(horzDistToTarget) < horzDistRange && Math.abs(forwardDistToGoal) < distanceRange
                    && Math.abs(facingAngle - e.getGyroCenteredOnGoal(facingAngle)) < 2) {
                e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                return true;
            } else {
                double calcInPerSec = calcInchesPerSec(maxTravelSpeed, calcDistToGo, 
                    maxTravelAcceleration, endingInchesPerSecond);
    
                e.assignRobotMotionAndHeadingField(calcHeading, calcInPerSec, facingAngle);
            }
        } else {
            e.assignRobotMotionAndHeadingField(0, 0, facingAngle);
        }
        return false;
    }

    /**
     * Drive to a specific location using any vision data. Goes backward or not at all.
     * 
     * @param facingAngle              Which way the bot will face while driving
     * @param targetDistanceInInches   How far in front of the target the bot wats
     *                                 to be
     * @param targetInchesRightOfTarget How far to the right the robot wants to be
     *                                 from the target
     * @param distanceRange            How precise the robot will try to be going
     *                                 forward/back
     * @param horzDistRange            How precise the robot will try to be going
     *                                 right/left
     * @param maxTravelSpeed           How fast the robot will move
     * @param maxTravelAcceleration    How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond    What the robot will slow down to before
     *                                 considering this action done
     * @param isTargetFound            Whether or not target is found, usually
     *                                 assigned by other methods
     * @param forwardDistToTarget      Current distance in fronnt of target, usually
     *                                 assigned by other methods
     * @param inchesRightOfTarget      Current distance to the right of target,
     *                                 usually assignned by other methods
     */
    public boolean driveBackToVisionTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfTarget, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, double endingInchesPerSecond, boolean isTargetFound,
            double forwardDistToTarget,
            double inchesRightOfTarget) {
        double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        if (isTargetFound && forwardDistToTarget < 0) {
            double horzDistToTarget = targetInchesRightOfTarget
                    - inchesRightOfTarget;
            double forwardDistToGoal = forwardDistToTarget - targetDistanceInInches;
            double calcDistToGo = Math.sqrt((Math.pow(horzDistToTarget, 2)) + Math.pow(forwardDistToGoal, 2));
            double calcHeading = gyroAngle + (90 - Math.toDegrees(Math.atan2(forwardDistToGoal, horzDistToTarget)));
            if (Math.abs(horzDistToTarget) < horzDistRange && Math.abs(forwardDistToGoal) < distanceRange
                    && Math.abs(facingAngle - e.getGyroCenteredOnGoal(facingAngle)) < 2) {
                e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                return true;
            } else {
                double calcInPerSec = calcInchesPerSec(maxTravelSpeed, calcDistToGo, 
                    maxTravelAcceleration, endingInchesPerSecond);
                e.assignRobotMotionAndHeadingField(calcHeading, calcInPerSec, facingAngle);
            }
        } else {
            e.assignRobotMotionAndHeadingField(0, 0, facingAngle);
        }
        return false;
    }

    /**
     *  Rotates the bot to a desired facing angle
     */ 
    public boolean rotateRobotToFacing(double facingAngle) {
        e.assignRobotMotionAndHeadingField(0, 0, facingAngle);
        if (RobotMath.isAbsoluteAngleDifferenceLessThan(facingAngle, robotState.gyroAngle, 0.6)) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Limits the acceleration of the robot based on the travel speed and max travel
     * acceleration.
     * 
     * @param goalInPerSec
     * @param maxTravelAcceleration
     */
    private double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        double maxAcceleration = robotState.commandElapsedTime * maxTravelAcceleration;
        if (goalInPerSec >= robotState.commandedVelocity) {
            calcInPerSec = Math.min(goalInPerSec, robotState.commandedVelocity + maxAcceleration);
        } else {
            calcInPerSec = Math.max(goalInPerSec, robotState.commandedVelocity - maxAcceleration);
        }
        return calcInPerSec;
    }

    public double getDistanceTraveled() {
        return calcDistTraveled;
    }

    public boolean driveToPickUpGamePiece(double travelAngle, double travelInchesPerSecond,
        double facingAngle, double maxTravelAcceleration, double distance,
        double endingInchesPerSecond, boolean allowAdjustForDistance,
        boolean isGamePieceFound, double gamePieceFacingAngle) {

        // If game piece is found, steer toward it, otherwies just drive straight
        if (isGamePieceFound ) {
            travelAngle = gamePieceFacingAngle;
            facingAngle = gamePieceFacingAngle;
        }

        return driveStraightWithFacing(travelAngle, travelInchesPerSecond, facingAngle,
            maxTravelAcceleration, distance, endingInchesPerSecond);
    }

    public void logChanges() {
        if (logTimer.get() < 3.0) {
            return; // Start logging changes after 3 seconds
        }

        boolean driverStationAttached = DriverStation.isDSAttached();

        if (driverStationAttached != lastDriverStationAttached) {
            rLog.print("Driver Station Attached: " + driverStationAttached + ", Voltage: " + e.getBatteryVoltage() +
            ", Amps: " + e.getTotalAmps());
        }
        lastDriverStationAttached = driverStationAttached;
    }
}