// LEDString.java - Controls the colors for a single LED String
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.*;
import frc.robot.RobotEnums.*;

public class LEDString {
    private int ledCount;
    private AddressableLED ledString;
    private AddressableLEDBuffer ledBuffer;

    LEDString (int port, int ledCount) {
        ledString = new AddressableLED(port);
        ledString.setLength(ledCount);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        ledString.setData(ledBuffer);
        ledString.start();
        
        this.ledCount = ledCount;
    }

    public void setLED(int ledNumber, Color color) {
        if (ledNumber >= 0 && ledNumber < ledCount) {
            ledBuffer.setLED(ledNumber, color);
        } 
    }

    public void setLED(int ledNumber, LEDcolor color) {
        Color rgb;
        if (color == LEDcolor.Red) {
            rgb = new Color(150, 0, 0);
        } else if (color == LEDcolor.DimRed) {
            rgb = new Color(20, 0, 0);
        } else if (color == LEDcolor.Blue) {
            rgb = new Color(0, 0, 255);
        } else if (color == LEDcolor.DimBlue) {
            rgb = new Color(0, 0, 20);
        } else if (color == LEDcolor.Green) {
            rgb = new Color(0, 150, 0);
        } else if (color == LEDcolor.GreenBlue) {
            rgb = new Color(0, 150, 70);
        } else if (color == LEDcolor.Yellow) {
            rgb = new Color(120, 120, 0);
        } else if (color == LEDcolor.Black) {
            rgb = new Color(0, 0, 0);
        } else if (color == LEDcolor.Purple) {
            rgb = new Color(91, 0, 127);
        } else if (color == LEDcolor.White) {
            rgb = new Color(100, 100, 100);
        } else if (color == LEDcolor.DimWhite) {
            rgb = new Color(15, 15, 15);
        } else if (color == LEDcolor.Orange) {
            rgb = new Color(255, 80, 0);
        } else if (color == LEDcolor.PastelRed) {
            rgb = new Color(255, 105, 97);
        } else if (color == LEDcolor.PastelYellow) {
            rgb = new Color(253, 253, 150);
        } else if (color == LEDcolor.PastelGreen) {
            rgb = new Color(119, 221, 119);
        } else if (color == LEDcolor.PastelBlue) {
            rgb = new Color(174, 198, 207);
        } else if (color == LEDcolor.PastelPurple) {
            rgb = new Color(195, 177, 225);
        } else {
            rgb = Color.kBlack;
        }
        setLED(ledNumber, rgb);
    }

    public void setLEDs(int startNumber, int endNumber, Color color) {
        for(int i = startNumber; i <= endNumber; i++) {
            setLED(i, color);
        }
    }

    public void setLEDs(int startNumber, int endNumber, LEDcolor color) {
        for(int i = startNumber; i <= endNumber; i++) {
            setLED(i, color);
        }
    }
    
    public void sendLEDBuffer() {
        ledString.setData(ledBuffer);
    }
}