package com.shprobotics.pestocore.devices;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class GamepadInterface {
    private final Gamepad gamepad;
    private final ArrayList<GamepadKeyInterface> keys;

    public GamepadInterface(Gamepad gamepad) {
        this.gamepad = gamepad;

        keys = new ArrayList<>();

        for (GamepadKey key: GamepadKey.values()) {
            this.keys.add(new GamepadKeyInterface(key.isPressed));
        }
    }

    public void update() {
        for (GamepadKeyInterface key: this.keys) {
            key.update(this.gamepad);
        }
    }

    public boolean isKeyDown(GamepadKey key) {
        return keys.get(key.idx).isKeyDown();
    }

    public boolean isKeyUp(GamepadKey key) {
        return keys.get(key.idx).isKeyUp();
    }

    public boolean isKey(GamepadKey key) {
        return keys.get(key.idx).isKey();
    }

    public double getTimeDown(GamepadKey key) {
        return keys.get(key.idx).getTimeDown();
    }

    public boolean isClicked(GamepadKey key, double maxSeconds) {
        return keys.get(key.idx).isClicked(maxSeconds);
    }

    public boolean isHeld(GamepadKey key, double minSeconds) {
        return keys.get(key.idx).isHeld(minSeconds);
    }
}
