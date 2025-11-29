package com.shprobotics.pestocore.processing;

import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalData;
import com.acmerobotics.dashboard.message.MessageCache;
import com.acmerobotics.dashboard.message.redux.CachableMessage;
import com.acmerobotics.dashboard.message.redux.SetData;

public class Cerebrum {
    public static void initialize() {
        MessageCache.initialize();
    }

    public static void printMessages(PestoTelemetry telemetry) {
        int i = 0;

        while (i < MessageCache.getSize()) {
            CachableMessage message = MessageCache.getElement(i);
            i++;

            if (message instanceof SetData) {
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
