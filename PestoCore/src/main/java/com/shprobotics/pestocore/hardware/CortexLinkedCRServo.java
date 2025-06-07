package com.shprobotics.pestocore.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.shprobotics.pestocore.processing.MotorCortex;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

public class CortexLinkedCRServo extends CachingCRServo {
    public CortexLinkedCRServo(@NonNull CRServo crServo) {
        super(crServo);
    }

    @Override
    public boolean setPowerResult(double power) {
        if (MotorCortex.motorsActivated)
            return super.setPowerResult(power);

        super.setPowerResult(0);
        return false;
    }
}
