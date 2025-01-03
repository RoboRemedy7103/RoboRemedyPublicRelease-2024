// LinearMapper.java - Calculate values using linear interpolation
package frc.robot;

import java.util.*;

public class LinearMapper {
    private int maxSize;

    LinearMapper() {
        maxSize = 0;
    }

    LinearMapper(int maxSize) {
        this.maxSize = maxSize;
    }

    private ArrayList<Double> inputArray = new ArrayList<Double>();
    private ArrayList<Double> outputArray = new ArrayList<Double>();
    private double inputOffset = 0;
    private double outputOffset = 0;

    public void withInputOffset(double inputOffset) {
        this.inputOffset = inputOffset;
    }

    public void withOutputOffset(double outputOffset) {
        this.outputOffset = outputOffset;
    }

    public void add(double input, double output) {
        if (maxSize == 0) {
            // If there is no max size, just add the new entry
            inputArray.add(input);
            outputArray.add(output);
        } else {
            if (inputArray.size() >= maxSize) {
                // If the max size has been reached, first move all of the existing elements
                // back one place (losing the oldest one), and then add the new entry to the end
                for (int i = 0; i < maxSize - 1; i++) {
                    inputArray.set(i, inputArray.get(i + 1));
                    outputArray.set(i, outputArray.get(i + 1));
                }
                inputArray.set(maxSize - 1, input);
                outputArray.set(maxSize - 1, output);
            } else {
                // If max size has not been reached, then just add the new entry
                inputArray.add(input);
                outputArray.add(output);
            }
        }
    }

    public double calculate(double in) {
        double out = 0;
        double input = in + inputOffset;
        if (inputArray.size() == 0) {
            return 0;
        }
        if (input <= inputArray.get(0)) {
            out = outputArray.get(0);
        } else if (input >= inputArray.get(inputArray.size() - 1)) {
            out = outputArray.get(outputArray.size() - 1);
        } else {
            boolean found = false;
            for (int i = 1; !found; i++) {
                if (input <= inputArray.get(i)) {
                    double rise = inputArray.get(i) - inputArray.get(i - 1);
                    double run = outputArray.get(i) - outputArray.get(i - 1);
                    double slope = rise / run;
                    out = ((input - inputArray.get(i - 1)) / slope) + outputArray.get(i - 1);
                    found = true;
                }
            }
        }
        return out + outputOffset;
    }

    public double getMaxInputValue() {
        if (inputArray.size() > 0)
            return inputArray.get(inputArray.size() - 1);
        else
            return 0;
    }

    public double getMaxOutputValue() {
        if (outputArray.size() > 0)
            return outputArray.get(outputArray.size() - 1);
        else
            return 0;
    }

    public String getArrayString() {
        String returnValue = "";
        for (int i = 0; i < inputArray.size(); i++) {
            returnValue += inputArray.get(i).toString() + ", " + outputArray.get(i).toString() + "; ";
        }
        return returnValue;
    }
}