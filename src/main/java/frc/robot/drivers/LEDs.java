package frc.robot.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.Level;

import static frc.robot.Constants.LEDs.*;

public class LEDs extends SubsystemBase {
    
    private final AddressableLED mLeds;
    private final AddressableLEDBuffer mBuffer;
    private Color selectedColor = Color.NA;

    /* Enum to keep track of currently selected color */
    private enum Color {
        NA, PURPLE, YELLOW
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

    /* Set the colors depending on the operators last selected scoring location */
    @Override
    public void periodic() {
        int scoringLocation = RobotContainer.operatorControls.getLastSelectedScoringLocation();
        Level level = RobotContainer.operatorControls.getLastSelectedLevel();
        boolean isCubeNode = scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8;
        if (isCubeNode && selectedColor != Color.PURPLE && level != Level.LOW) {
            setColor(PURPLE[0], PURPLE[1], PURPLE[2]);
            selectedColor = Color.PURPLE;
        }

        if (!isCubeNode && selectedColor != Color.YELLOW && level != Level.LOW) {
            setColor(YELLOW[0], YELLOW[1], YELLOW[2]);
            selectedColor = Color.YELLOW;
        }

        if (level == Level.LOW && selectedColor != Color.PURPLE) {
            setColor(PURPLE[0], PURPLE[1], PURPLE[2]);
            selectedColor = Color.PURPLE;
        }
    }
}
