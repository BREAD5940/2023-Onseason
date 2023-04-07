package frc.robot.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.Superstructure.Level;

import static frc.robot.Constants.LEDs.*;

import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {
    
    private final AddressableLED mLeds;
    private final AddressableLEDBuffer mBuffer;
    private Color selectedColor = Color.NA;
    private boolean dangerBlink = false;
    private boolean happyBlink = false;


    /* Enum to keep track of currently selected color */
    private enum Color {
        NA, PURPLE, YELLOW, GREEN
    }
    
    /* Constructs the LEDs object */
    public LEDs(int port, int bufferLen) {
        // Construct the LEDs
        mLeds = new AddressableLED(port);

        // Construct the buffer
        mBuffer = new AddressableLEDBuffer(bufferLen);
        mLeds.setLength(mBuffer.getLength());

        // Set the data
        mLeds.setData(mBuffer);
        mLeds.start();
    }

    /* Sets the color of the LEDs */
    private void setColor(int r, int g, int b) { 
        for (int i = 0; i < mBuffer.getLength(); i++) {
            mBuffer.setRGB(i, r, g, b);
        }
        mLeds.setData(mBuffer);
    }

    public void setDangerBlink(boolean on){
        dangerBlink = on;
    }

    public void setHappyBlink(boolean on){
        happyBlink = on;
    }

    /* Set the colors depending on the operators last selected scoring location */
    @Override
    public void periodic() {
        int scoringLocation = RobotContainer.operatorControls.getLastSelectedScoringLocation();
        double blinkTime = BreadUtil.getFPGATimeSeconds() * 10.0;
        boolean blinkOn = ((int) blinkTime) % 2 == 0;
        boolean dangerBlinkOn = ((int) blinkTime) % 0.5 == 0;
        Level level = RobotContainer.operatorControls.getLastSelectedLevel();
        boolean isCubeNode = scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8;
        if (dangerBlink && blinkOn){
            setColor(255, 0, 0);
        } else if (dangerBlink && !blinkOn){
            setColor(0, 0, 0);
        } else if (happyBlink && blinkOn){
            setColor(0, 255, 0);
        } else if (happyBlink && !blinkOn){
            setColor(0, 0, 0);
        } else if (blinkOn && RobotContainer.superstructure.hasGampiece()) {
            setColor(GREEN[0], GREEN[1], GREEN[2]);
            selectedColor = Color.GREEN;
        } else if (!blinkOn && RobotContainer.superstructure.hasGampiece()) {
            setColor(0, 0, 0);
            selectedColor = Color.NA;
        } else if (isCubeNode && selectedColor != Color.PURPLE && level != Level.LOW) {
            setColor(PURPLE[0], PURPLE[1], PURPLE[2]);
            selectedColor = Color.PURPLE;
        } else if (!isCubeNode && selectedColor != Color.YELLOW && level != Level.LOW) {
            setColor(YELLOW[0], YELLOW[1], YELLOW[2]);
            selectedColor = Color.YELLOW;
        } else if (level == Level.LOW && selectedColor != Color.PURPLE) {
            setColor(PURPLE[0], PURPLE[1], PURPLE[2]);
            selectedColor = Color.PURPLE;
        }

        Logger.getInstance().recordOutput("SelectedLEDColor", selectedColor.toString());
    }
}
