package com.shprobotics.pestocore.devices;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

class GamepadKeyInterface {
    private final Function<Gamepad, Boolean> isPressed;
    private boolean keyDown;
    private boolean key;
    private boolean keyUp;
    private double lastPressed;

    public GamepadKeyInterface(Function<Gamepad, Boolean> isPressed) {
        this.isPressed = isPressed;
        this.keyDown = false;
        this.key = false;
        this.keyUp = false;
        this.lastPressed = Double.POSITIVE_INFINITY;
    }

    public void update(Gamepad gamepad) {
        this.keyDown = !this.key && this.isPressed.apply(gamepad);
        this.keyUp = this.key && !this.isPressed.apply(gamepad);
        this.key = this.isPressed.apply(gamepad);
        if (keyDown)
            this.lastPressed = System.nanoTime() / 1E9;
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

    public double getTimeDown() {
        if (!this.key)
            return 0.0;

        return (System.nanoTime() / 1E9) - this.lastPressed;
    }

    public boolean isClicked(double maxSeconds) {
        return this.keyUp && (System.nanoTime() / 1E9) - this.lastPressed <= maxSeconds;
    }

    public boolean isHeld(double minSeconds) {
        return this.keyUp && (System.nanoTime() / 1E9) - this.lastPressed >= minSeconds;
    }
}
