// PhotonVisionCamera.java - Controls a PhotonVision camera on an Orange Pi
package frc.robot;

import org.photonvision.*;
import org.photonvision.targeting.*;

public class PhotonVisionCamera {

    public static class TargetInfo {
        public boolean isFound;
        public int id;
        public double yaw;
        public double pitch;
        public double timeStamp;
    }

    private PhotonCamera camera;
    private RoboLog rLog;
    private PhotonPipelineResult lastResult = new PhotonPipelineResult();

    PhotonVisionCamera (String cameraName, RoboLog rLog) {
        camera = new PhotonCamera(cameraName);
        this.rLog = rLog;
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public void setDriverMode(boolean driveMode) {
        if (camera.isConnected()) {
            camera.setDriverMode(driveMode);
            rLog.print("Driver Mode:" + driveMode);
        }
    }

    public void takeNoteSnapshot() {
        camera.takeInputSnapshot();
        camera.takeOutputSnapshot();
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public void findSpecificAprilTag(int targetID, TargetInfo info) {
        info.isFound = false;
        info.id = 0;
        info.timeStamp = Double.MAX_VALUE;
        info.yaw = Double.MAX_VALUE;
        info.pitch = Double.MAX_VALUE;
        if (camera.isConnected()) {
            var result = getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == targetID) {
                        info.isFound = true;
                        info.yaw = target.getYaw();
                        info.pitch = target.getPitch();
                        info.timeStamp = result.getTimestampSeconds();
                        info.id = targetID;
                    }
                }
            }
        }
    }

    public void getBestShapeByYaw(TargetInfo info) {
        info.isFound = false;
        info.yaw = Double.MAX_VALUE;
        info.pitch = Double.MAX_VALUE;
        double bestYaw = Double.MAX_VALUE;
        if (camera.isConnected()) {
            var result = getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (Math.abs(target.getYaw()) < bestYaw) {
                        info.isFound = true;
                        info.yaw = target.getYaw();
                        info.pitch = target.getPitch();
                        info.timeStamp = result.getTimestampSeconds();
                        bestYaw = Math.abs(info.yaw);
                    }
                }
            }
        }
    }

    public void getBestShapeByPitch(TargetInfo info) {
        info.isFound = false;
        info.yaw = Double.MAX_VALUE;
        info.pitch = Double.MAX_VALUE;
        double bestPitch = Double.MAX_VALUE;
        if (camera.isConnected()) {
            var result = getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (Math.abs(target.getPitch()) < bestPitch) {
                        info.isFound = true;
                        info.yaw = target.getYaw();
                        info.pitch = target.getPitch();
                        info.timeStamp = result.getTimestampSeconds();
                        bestPitch = Math.abs(info.pitch);
                    }
                }
            }
        }
    }

    private PhotonPipelineResult getLatestResult() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            lastResult = results.get(results.size() - 1);
        }
        return lastResult;
    }
}
