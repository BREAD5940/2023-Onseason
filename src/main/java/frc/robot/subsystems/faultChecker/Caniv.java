package frc.robot.subsystems.faultChecker;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.LineNumberReader;
import org.littletonrobotics.junction.Logger;

public class Caniv {
    Runtime rt;
    public int percentBusUtilization = 0;

    public Caniv() {
        rt = Runtime.getRuntime();
    }

    public void update() {
        String[] commands = { "system.exe", "-get t" };
        Process proc;
        try {
            proc = rt.exec(commands);
            LineNumberReader stdInput = new LineNumberReader(new InputStreamReader(proc.getInputStream())); 
            stdInput.setLineNumber(14);
            String utilizationLine = stdInput.readLine();
            String utilString = utilizationLine.substring(utilizationLine.lastIndexOf(":") + 1);
            percentBusUtilization = Integer.parseInt(utilString);  
            Logger.getInstance().recordOutput("CanivoreUtilization", percentBusUtilization);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
