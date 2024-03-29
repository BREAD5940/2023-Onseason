package frc.robot.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OperatorControls;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.LEDs.*;

import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {
    
    private final AddressableLED mLeds;
    private final AddressableLEDBuffer mBuffer;
    private Color selectedColor = Color.NA;

    /* Enum to keep track of currently selected color */
    private enum Color {
        NA, PURPLE, YELLOW, BLUE, TURQUOISE
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
        double blinkTime = BreadUtil.getFPGATimeSeconds() * 10.0;
        boolean blinkOn = ((int) blinkTime) % 2 == 0;
        Level level = RobotContainer.operatorControls.getLastSelectedLevel();
        boolean isCubeNode = RobotContainer.operatorControls.getLastSelectedGamePiece() == GamePiece.CUBE;
        if (blinkOn && RobotContainer.superstructure.hasGampiece()) {
            setColor(BLUE[0], BLUE[1], BLUE[2]);
            selectedColor = Color.BLUE;
        } else if (!blinkOn && RobotContainer.superstructure.hasGampiece()) {
            setColor(0, 0, 0);
            selectedColor = Color.NA;
        } else if (isCubeNode && selectedColor != Color.PURPLE && level != Level.LOW) {
            setColor(PURPLE[0], PURPLE[1], PURPLE[2]);
            selectedColor = Color.PURPLE;
        } else if (Swerve.alignedChargeStation == true && Swerve.aligningChargeStation == true) {
            setColor(TURQUOISE[0], TURQUOISE[1], TURQUOISE[2]);
            selectedColor = Color.TURQUOISE;
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
