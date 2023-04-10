//0231 (DONT DELETE I NEED THIS COMMENT)

package frc.robot.subsystems.faultChecker;

import java.io.IOException;
import java.net.InetAddress;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.SerialPort;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.GregorianCalendar;

public class FaultChecker extends Thread {
    int loopCounter = 0;
    public int radioErrCount = 0;
    // order limelight, orangepi1, orangepi2
    public int[] ErrCounts = {0, 0, 0};
    public int limelightErrCount = 0;
    public int orangepi1ErrCount = 0;
    public int orangepi2ErrCount = 0;
    private String[] swerveOrder = {"FL", "FR", "BL", "BR"};

    double allowableCANerror = 0.01;

    String priorityCANerror = "OK";
    String priorityETHerror = "OK";
    String priorityTEMPerror = "OK";

    InetAddress radio;
    InetAddress limelight;
    InetAddress orangepi1;
    InetAddress orangepi2;

    Caniv caniv = new Caniv();

    SerialPort serialMXP = new SerialPort(115200, SerialPort.Port.kMXP);

    public FaultChecker() {
        serialMXP.disableTermination();
        try {
            radio = InetAddress.getByName(Constants.Network.radio);
            limelight = InetAddress.getByName(Constants.Network.limelight);
            orangepi1 = InetAddress.getByName(Constants.Network.orangepi1);
            orangepi2 = InetAddress.getByName(Constants.Network.orangepi2);
        } catch (Exception e) {
            System.out.println(e);
        }

    }

	@Override
    public void run() {
		while (true) {
			System.out.println("---------- ERRORS ----------");
			priorityCANerror = "OK";
			priorityETHerror = "OK";

			
			ArrayList<Pair<Boolean, String>> ethernetErrors = new ArrayList<>();

			try {
				ethernetErrors.add(new Pair<Boolean,String>(limelight.isReachable(20), "Limelight"));
			} catch (IOException e) {
				e.printStackTrace();
			}
			try {
				ethernetErrors.add(new Pair<Boolean,String>(orangepi1.isReachable(20), "Orangepi1"));
			} catch (IOException e) {
				e.printStackTrace();
			}
			try {
				ethernetErrors.add(new Pair<Boolean,String>(orangepi2.isReachable(20), "Orangepi2"));
			} catch (IOException e) {
				e.printStackTrace();
			}

			for (int i = 0; i < ethernetErrors.size(); i++) {
				Pair<Boolean,String> ethernetError = ethernetErrors.get(i);
				if (!ethernetError.getFirst()) {
					ErrCounts[i]++;
					Logger.getInstance().recordOutput("Ethernet/" + ethernetError.getSecond(), false);
					System.out.println("Ethernet/" + ethernetError.getSecond() + ": false");
					priorityETHerror = ethernetError.getSecond()+ " NC";
				} else {
					Logger.getInstance().recordOutput("Ethernet/" + ethernetError.getSecond(), true);
					System.out.println("Ethernet/" + ethernetError.getSecond() + ": true");
				}
			}

			long finish = 0;
			long start = new GregorianCalendar().getTimeInMillis();

			try {
				if (!radio.isReachable(5000)) {
					radioErrCount++;
					Logger.getInstance().recordOutput("Ethernet/Radio", false);
					System.out.println("Ethernet/Radio: " + "false");
					priorityETHerror = "Radio NC";
					Logger.getInstance().recordOutput("Ethernet/RadioTime", 5000);
					System.out.println("Ethernet/RadioTime: " + "5000");
				} else {
					Logger.getInstance().recordOutput("Ethernet/Radio", true);
					System.out.println("Ethernet/Radio: " + "true");
					finish = new GregorianCalendar().getTimeInMillis();
					Logger.getInstance().recordOutput("Ethernet/RadioTime", finish - start);
					System.out.println("Ethernet/RadioTime: " + (finish - start));
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
			

			ArrayList<Pair<Boolean, String>> CANerrors = new ArrayList<>();
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.climber.getClimberErrorConc() > allowableCANerror, "ClimberErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getDeployErrorConc() > allowableCANerror, "DeployErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getRollerErrorConc() > allowableCANerror, "RollerErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getFollowerErrorConc() > allowableCANerror, "FollowerErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getLeaderErrorConc() > allowableCANerror, "LeaderErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getArmAzimuthErrorConc() > allowableCANerror, "ArmAzimuthErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getArmErrorConc() > allowableCANerror, "ArmErrorConc"));
			CANerrors.add(new Pair<Boolean, String>(RobotContainer.superstructure.getEndEffectorErrorConc() > allowableCANerror, "EndEffectorErrorConc"));

			RobotContainer.superstructure.resetError();
			
			for (int i = 0; i < 4; i++) {
				CANerrors.add(new Pair<Boolean, String>(RobotContainer.swerve.getDriveErrorConc(i) > allowableCANerror, "DriveErrorConc " + swerveOrder[i]));
				CANerrors.add(new Pair<Boolean, String>(RobotContainer.swerve.getSteerErrorConc(i) > allowableCANerror, "SteerErrorConc " + swerveOrder[i]));
				CANerrors.add(new Pair<Boolean, String>(RobotContainer.swerve.getAzimuthErrorConc(i) > allowableCANerror, "AzimuthErrorConc " + swerveOrder[i]));
			}
			RobotContainer.swerve.resetError();        
			
			// addeds it to the display on the robot
			for (Pair<Boolean, String> CANerror : CANerrors) {
				if (CANerror.getFirst()) {
					System.out.println(CANerror.getSecond() + ": HAS ERROR");
					priorityCANerror = CANerror.getSecond().replace("ErrorConc", ""); // removes the ErrorConc for space reasons
				} else {
					System.out.println(CANerror.getSecond() + ": is ok");
				}
			}

			loopCounter++;
			if (loopCounter % 100 == 0) {
				CANStatus status = new CANStatus();
				CANJNI.getCANStatus(status);
				// serialMXP.writeString("<0,0,0,0>");
				serialMXP.writeString("<" + "stuff will go here" + "," + ((int) (status.percentBusUtilization * 100)) + "," + priorityCANerror + "_" + priorityETHerror + "," + "?>");
			}

			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			System.out.print("---------- END OF ERRORS ----------");
		}
	}
}
