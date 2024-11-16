package com.shprobotics.pestocore.devices;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

class GamepadKeyInterface {
    private final Function<Gamepad, Boolean> isPressed;
    private boolean keyDown;
    private boolean key;
    private boolean keyUp;

    public GamepadKeyInterface(Function<Gamepad, Boolean> isPressed) {
        this.isPressed = isPressed;
        this.keyDown = false;
        this.key = false;
        this.keyUp = false;
    }

    public void update(Gamepad gamepad) {
        this.keyDown = !this.key && this.isPressed.apply(gamepad);
        this.keyUp = this.key && !this.isPressed.apply(gamepad);
        this.key = this.isPressed.apply(gamepad);
    }

    public boolean isKeyDown() {
        return this.keyDown;
    }

    public boolean isKeyUp() {
        return this.keyUp;
    }

    public boolean isKey() {
        return this.key;
    }
}
