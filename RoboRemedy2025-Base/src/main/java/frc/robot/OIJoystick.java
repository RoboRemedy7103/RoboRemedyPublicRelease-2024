package frc.robot;

import edu.wpi.first.wpilibj.*;

/**
 * A class to represent a specific joystick.
 * Used to auto-assign joysticks based on connected controllers.
 * 
 */
class OIJoystick {
    private String joystickName = "";
    private String joystickStatus = "Null";
    private int joystickType = 0;//Method for this returns an int, but pulls from an enum. Must find what enum is.
    private int joystickButtonCount = 0;
    private int joystickAxisCount = 0;
    private int joystickPort = 0;

    OIJoystick(int Port) {
        joystickPort = Port;
        joystickName = DriverStation.getJoystickName(joystickPort);
        joystickType = DriverStation.getJoystickType(joystickPort);
        joystickButtonCount = DriverStation.getStickButtonCount(joystickPort);
        joystickAxisCount = DriverStation.getStickAxisCount(joystickPort);
    }

    void refreshJoystickInfo() {
        joystickName = DriverStation.getJoystickName(joystickPort);
        joystickType = DriverStation.getJoystickType(joystickPort);
        joystickButtonCount = DriverStation.getStickButtonCount(joystickPort);
        joystickAxisCount = DriverStation.getStickAxisCount(joystickPort);
    }
    
    void setJoystickStatus(String status) {
        joystickStatus = status;
    }

    boolean isConnected() {
        return DriverStation.isJoystickConnected(joystickPort);
    }

    String getJoystickStatus() {
        return joystickStatus;
    }

    int getJoystickPort() {
        return joystickPort;
    }

    String getJoystickName() {
        return joystickName;
    }

    int getJoystickType() {
        return joystickType;
    }

    int getJoystickButtonCount() {
        return joystickButtonCount;
    }

    int getJoystickAxisCount() {
        return joystickAxisCount;
    }
}
