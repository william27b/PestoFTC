package com.shprobotics.pestocore.processing;

public class HardwareException extends RuntimeException {
    public HardwareException(String message) {
        super(message);
    }

    public HardwareException() {
        super("HardwareMap is not initialized");
    }
}
