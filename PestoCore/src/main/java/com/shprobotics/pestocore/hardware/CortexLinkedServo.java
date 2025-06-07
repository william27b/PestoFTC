package com.shprobotics.pestocore.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.processing.MotorCortex;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class CortexLinkedServo extends CachingServo {
    public CortexLinkedServo(@NonNull Servo servo) {
        super(servo);
    }

    @Override
    public boolean setPositionResult(double position) {
        if (MotorCortex.motorsActivated)
            return super.setPositionResult(position);

        MotorCortex.ServoCommands.disableServo(this);
        return false;
    }
}
