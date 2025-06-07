package com.shprobotics.pestocore.tuners;

import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalData;
import com.acmerobotics.dashboard.config.variable.NumericalSelector;
import com.acmerobotics.dashboard.config.variable.QualitativeData;
import com.acmerobotics.dashboard.config.variable.QualitativeSelector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.PestoTelemetry;

@TeleOp(name = "Telemetry Test", group = "Pesto Tuners")
public class TelemetryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        if (isStopRequested())
            return;

        NumericalData targetVelocitySelector = new NumericalData("targetVelocity", 30.0);
        targetVelocitySelector.setColor(DataItem.MessageColor.INFO);
        targetVelocitySelector.setUnit("inches / s");

        NumericalSelector a = new NumericalSelector("targetVelocity", 30.0, 0.0, 60.0);
        a.setColor(DataItem.MessageColor.INFO);
        a.setUnit("inches / s");

        QualitativeSelector b = new QualitativeSelector("targetVelocity", "i don't know", new String[]{
                "tell me wassup",
                "not my problem",
                "3",
        });
        b.setColor(DataItem.MessageColor.NEGATIVE);

        QualitativeSelector c = new QualitativeSelector("targetVelocity", "write here");
        b.setColor(DataItem.MessageColor.POSITIVE);

        QualitativeData d = new QualitativeData("this is", "a line");
        d.setColor(DataItem.MessageColor.NEUTRAL);

        pestoTelemetry.addToDash(targetVelocitySelector);
        pestoTelemetry.addToDash(a);
        pestoTelemetry.addToDash(b);
        pestoTelemetry.addToDash(c);
        pestoTelemetry.addToDash(d);
        pestoTelemetry.update();

        waitForStart();

        long lastNano = 0;

        while (opModeIsActive()) {
            pestoTelemetry.update();

//            double refreshTime = (System.nanoTime() - lastNano) / 1E9;
//            lastNano = System.nanoTime();
//
//            NumericalData hertz = new NumericalData("Refresh time", refreshTime);
//            hertz.setUnit("seconds");
//            hertz.setColor(DataItem.MessageColor.NEGATIVE);
//
//            NumericalData fps = new NumericalData("fps", 1 / refreshTime);
//            fps.setUnit("frames / s");
//            fps.setColor(DataItem.MessageColor.POSITIVE);
//
//            pestoTelemetry.addToDash(hertz);
//            pestoTelemetry.addToDash(fps);
//            pestoTelemetry.update();
        }
    }
}
