package com.shprobotics.pestocore.hardware;

import static com.shprobotics.pestocore.hardware.ControlType.ALL;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.shprobotics.pestocore.processing.MotorCortex;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class CortexLinkedMotor extends CachingDcMotorEx {
    private ControlType control;
    private String motorName = null;

    public CortexLinkedMotor(@NonNull DcMotorEx dcMotorEx) {
        super(dcMotorEx);
        control = ALL;
    }

    public void setName(String motorName) {
        this.motorName = motorName;
    }

    public String getName() {
        return motorName;
    }

    public void setControl(ControlType control) {
        this.control = control;
    }

    @Override
    public boolean setPowerResult(double power) {
        if (control != ALL)
            return false;

        if (MotorCortex.motorsActivated)
            return super.setPowerResult(power);

        super.setPowerResult(0);
        return false;
    }

    public boolean setPowerResult(double power, ControlType control) {
        if (this.control != control)
            return false;

        return this.setPowerResult(power);
    }
}
