package frc.robot.commons;

import java.util.function.DoubleSupplier;

public class TriggerButton {

    private final DoubleSupplier trigger; 
    private boolean pressed = false;

    public TriggerButton(DoubleSupplier trigger) {
        this.trigger = trigger;
    }

    // Returns whether the trigger was pressed
    public boolean whenPressed() {
        boolean rPressed = pressed;
        pressed = false;
        return rPressed;
    }


    // Returns whether the trigger was pressed
    public boolean whilePressed() {
        boolean rPressed = pressed;
        pressed = false;
        return rPressed;
    }

    // Returns the current value of the trigger if its being pressed
    public double whileTrue(double deadzone) {
        double triggerVal = trigger.getAsDouble(); 
        if (triggerVal < deadzone) return 0;
        else return triggerVal;
    }

    // Returns the current value of the trigger if its being pressed
    public double whenTrue(double deadzone) {
        double triggerVal = trigger.getAsDouble(); 
        if (triggerVal < deadzone) return 0;
        else return triggerVal;
    }

    /** This method must be called periodically for the trigger to work as intended */
    public void onLoop() {
        if (!pressed && trigger.getAsDouble() > 0.1) {
            pressed = true;
        }
    }
    
}
