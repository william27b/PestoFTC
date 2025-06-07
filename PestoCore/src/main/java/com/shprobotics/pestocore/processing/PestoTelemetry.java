package com.shprobotics.pestocore.processing;

import com.acmerobotics.dashboard.PestoDashCore;
import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalData;
import com.acmerobotics.dashboard.config.variable.QualitativeData;

public class PestoTelemetry {
    public void addToDash(String caption, String value) {
        PestoDashCore.addItem(new QualitativeData(
                caption,
                value
        ));
    }

    public void addToDash(String caption, double value) {
        PestoDashCore.addItem(new NumericalData(
                caption,
                value
        ));
    }

    public void addToDash(DataItem item) {
        PestoDashCore.addItem(item);
    }

    public void clear() {
        PestoDashCore.clear();
    }

    public void clearAll() {
        PestoDashCore.clearAll();
    }

    public void update() {
        PestoDashCore.update();
    }
}
