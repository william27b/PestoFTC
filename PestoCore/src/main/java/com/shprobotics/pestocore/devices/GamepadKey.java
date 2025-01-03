package com.shprobotics.pestocore.devices;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

public enum GamepadKey {
    A (0, gamepad -> gamepad.a),
    B (1, gamepad -> gamepad.b),
    X (2, gamepad -> gamepad.x),
    Y (3, gamepad -> gamepad.y),

    DPAD_DOWN (4, gamepad -> gamepad.dpad_down),
    DPAD_RIGHT (5, gamepad -> gamepad.dpad_right),
    DPAD_LEFT (6, gamepad -> gamepad.dpad_left),
    DPAD_UP (7, gamepad -> gamepad.dpad_up),

    LEFT_BUMPER (8, gamepad -> gamepad.left_bumper),
    RIGHT_BUMPER (9, gamepad -> gamepad.right_bumper),

    LEFT_STICK_BUTTON (10, gamepad -> gamepad.left_stick_button),
    RIGHT_STICK_BUTTON (11, gamepad -> gamepad.right_stick_button),
    LEFT_TRIGGER(12, gamepad -> gamepad.left_trigger > 0.1),
    RIGHT_TRIGGER(13, gamepad -> gamepad.right_trigger > 0.1),


    TOUCHPAD (14, gamepad -> gamepad.touchpad);

    GamepadKey(int idx, Function<Gamepad, Boolean> isPressed) {
        this.idx = idx;
        this.isPressed = isPressed;
    }

    public final int idx;
    public final Function<Gamepad, Boolean> isPressed;
}
