package com.shprobotics.pestocore.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
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

    public void setPwmRange(PwmControl.PwmRange range)
    {
        ((ServoControllerEx)this.getController()).setServoPwmRange(this.getPortNumber(), range);
    }

    public PwmControl.PwmRange getPwmRange()
    {
        return ((ServoControllerEx)this.getController()).getServoPwmRange(this.getPortNumber());
    }

    public void setPwmEnable()
    {
        ((ServoControllerEx)this.getController()).setServoPwmEnable(this.getPortNumber());
    }

    public void setPwmDisable()
    {
        ((ServoControllerEx)this.getController()).setServoPwmDisable(this.getPortNumber());
    }

    public boolean isPwmEnabled()
    {
        return ((ServoControllerEx)this.getController()).isServoPwmEnabled(this.getPortNumber());
    }
}
