package com.shprobotics.pestocore.processing;

import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalData;
import com.acmerobotics.dashboard.message.MessageCache;
import com.acmerobotics.dashboard.message.redux.CachableMessage;
import com.acmerobotics.dashboard.message.redux.SetData;
import com.acmerobotics.dashboard.message.redux.SetMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cerebrum {
    public static void initialize() {
        MessageCache.initialize();
    }

    public static void printMessages(Telemetry telemetry) {
        int i = 0;

        while (i < MessageCache.getSize()) {
            CachableMessage message = MessageCache.getElement(i);
            i++;

            if (!(message instanceof SetMotor))
                continue;

            SetMotor setMotor = (SetMotor) message;
            telemetry.addData(setMotor.getName(), setMotor.getPower());
        }
    }

    public static void printMessages(PestoTelemetry telemetry) {
        int i = 0;

        while (i < MessageCache.getSize()) {
            CachableMessage message = MessageCache.getElement(i);
            i++;

            if (message instanceof SetMotor) {
                SetMotor setMotor = (SetMotor) message;
                telemetry.addToDash(setMotor.getName(), setMotor.getPower());
            } else if (message instanceof SetData) {
                SetData setData = (SetData) message;

                for (DataItem item: setData.getData()) {
                    if (item instanceof NumericalData)
                        telemetry.addToDash(item.getCaption(), (double) (item.getValue()));
                    else
                        telemetry.addToDash(item.getCaption(), item.getValue().toString());
                }
            }
        }
    }
}
