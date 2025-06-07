package com.shprobotics.pestocore.processing;

/*
Processes sensory information like touch, temperature, and pain
 */

import com.shprobotics.pestocore.hardware.Sensor;

import java.util.ArrayList;

public class ParietalLobe {
    private static ArrayList<Sensor> sensors;
    public static int dataSize;

    public void initialize() {
        sensors = new ArrayList<>();
        dataSize = 0;
    }

    public void addSensor(Sensor sensor) {
        sensors.add(sensor);
        dataSize += sensor.getDataSize();
    }

    public double[] getData() {
        double[] data = new double[dataSize];
        int current = 0;

        for (Sensor sensor: sensors) {
            double[] sensorData = sensor.getData();

            for (double channel: sensorData) {
                if (current >= dataSize)
                    throw new HardwareException("Sensor.getDateSize() returning false or non-constant dataSize");

                data[current] = channel;
                current++;
            }
        }

        return data;
    }
}
