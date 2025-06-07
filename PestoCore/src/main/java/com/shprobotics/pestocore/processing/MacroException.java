package com.shprobotics.pestocore.processing;

public class MacroException extends RuntimeException {
  public MacroException(String message) {
    super(message);
  }

  public MacroException() {
    super("Macro alias does not exist");
  }
}
