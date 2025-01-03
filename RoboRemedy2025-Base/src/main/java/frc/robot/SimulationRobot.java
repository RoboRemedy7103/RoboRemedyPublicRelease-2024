// SimulationRobot.java - Code used to simulate robot
package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.MeasureConvert.*;
import org.photonvision.simulation.*;

import java.util.*;

public class SimulationRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private RobotState robotState;
    private Field2d simulationField = new Field2d();
    private VisionSystemSim visionSim;
    private PhotonCameraSim aprilTagCameraSim;
    private PhotonCameraSim noteCameraSim; 

    private Pose2d robotPose;

    private FieldObject2d noteDisplay;
    private ArrayList<Pose2d> notePoses;
    
    // Fields used by robotState during simulation
    private double batteryVoltage;

    public SimulationRobot(Electronics e, RoboLog rLog, Action act, OI oi,
        RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
            
        if (this.rLog == null) {
            System.out.println("Warning: SimulationRobot.rLog is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: SimulationRobot.robotState is null");
        }
        if (this.act == null) {
            rLog.print("Warning: SimulationRobot.act is null");
        }
        if (this.e == null) {
            rLog.print("Warning: SimulationRobot.e is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: SimulationRobot.oi is null");
        }

        rLog.print("Simulation Started");

        // Add robot to field
        robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        simulationField.setRobotPose(robotPose);
        SmartDashboard.putData("Field", simulationField);
        
        // Cameras
        visionSim = new VisionSystemSim("main"); 
        SimCameraProperties aprilTagCameraProp = new SimCameraProperties();
        aprilTagCameraProp.setFPS(20);
        aprilTagCameraProp.setAvgLatencyMs(35);
        aprilTagCameraSim = new PhotonCameraSim(e.getMainAprilTagCamera().getPhotonCamera(),
            aprilTagCameraProp, 1.0, 5);
        visionSim.addCamera(aprilTagCameraSim,new Transform3d());
        SimCameraProperties noteCameraProp = new SimCameraProperties();
        noteCameraProp.setFPS(20);
        noteCameraProp.setAvgLatencyMs(35);
        noteCameraSim = new PhotonCameraSim(e.getGamePieceCamera().getPhotonCamera(),
            noteCameraProp, 1.0, 5);
        visionSim.addCamera(noteCameraSim,new Transform3d());
        visionSim.update(robotPose);

        // Objects on the field
        noteDisplay = simulationField.getObject("Note");
        notePoses = new ArrayList<Pose2d>();
        notePoses.add(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0)));
        noteDisplay.setPoses(notePoses);
    }   

    public void beforeRobotStatePeriodic() {
        // Calculate new robot position and heading
        double travelAngleDegrees;
        if (robotState.isRedAlliance) {
            travelAngleDegrees = e.getLastTravelAngleDegrees() - 180;
        } else {
            travelAngleDegrees = e.getLastTravelAngleDegrees();
        }
        Pose2d currentPose = simulationField.getRobotPose();
        double distanceMoved = robotState.distanceMoved;
        double xChange = Math.cos(Math.toRadians(travelAngleDegrees)) * distanceMoved;
        double yChange = Math.sin(Math.toRadians(travelAngleDegrees)) * distanceMoved * -1;
        double newPositionX = MeasureConvert.convertMeasurement(xChange, Unit.Inch, Unit.Meter) + currentPose.getX();
        double newPositionY = MeasureConvert.convertMeasurement(yChange, Unit.Inch, Unit.Meter) + currentPose.getY();
        double angleChange = robotState.commandElapsedTime * e.getLastTravelRotationDegreesPerSecond();
        double newAngle = currentPose.getRotation().getDegrees() - angleChange;
        if ((xChange != 0) || (yChange != 0) || (angleChange != 0)) {
            robotPose = new Pose2d(newPositionX, newPositionY, Rotation2d.fromDegrees(newAngle));
            simulationField.setRobotPose(robotPose);
            if (robotState.isRedAlliance) {
                e.setSimulationGyro(- newAngle - 180);
            } else {
                e.setSimulationGyro(- newAngle);
            }
            
        } else {
            robotPose = currentPose;
        }

        // Battery Voltage
        if (robotState.robotElapsedTime < 60.0) {
            batteryVoltage = 12.5;
        } else {
            batteryVoltage = 12.0;
        }

        // Photon Vision
        visionSim.update(robotPose);
    }

    // Methods used by robotState during simulation
    public double getBatteryVoltage() {
        return batteryVoltage;
    }
}
