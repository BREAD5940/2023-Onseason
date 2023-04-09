package frc.robot.subsystems.faultChecker;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.LineNumberReader;
import org.littletonrobotics.junction.Logger;

public class Caniv{
    Runtime rt;
    public int percentBusUtilization = 0;
    public String utilString = "";
    public String utilizationLine = "";

    public Caniv() {
        rt = Runtime.getRuntime();
    }

    public void update() {
        String[] commands = { "caniv -d * -i"};
        Process proc;
        try {
            proc = rt.exec(commands);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            LineNumberReader stdInput = new LineNumberReader(new InputStreamReader(proc.getInputStream())); 
            // stdInput.setLineNumber(14);
            String utilizationLine = stdInput.readLine();
            utilString = utilizationLine.substring(utilizationLine.lastIndexOf(":") + 1);
            percentBusUtilization = Integer.parseInt(utilString);
            Logger.getInstance().recordOutput("CanivoreUtilization", percentBusUtilization);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
