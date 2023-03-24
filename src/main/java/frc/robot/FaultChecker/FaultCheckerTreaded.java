//0231 (DONT DELETE I NEED THIS COMMENT)

package frc.robot.FaultChecker;

import java.net.InetAddress;
import java.util.Arrays;
import java.util.List;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Network;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import org.littletonrobotics.junction.Logger;

public class FaultCheckerTreaded extends Thread {
	int loopCounter = 0;
	public int radioErrCount = 0;
	public int limelightErrCount = 0;
	public int orangepi1ErrCount = 0;
	public int orangepi2ErrCount = 0;

	List<Double> deviceErrorsDabus = Arrays.asList(new Double[17]); // 18 can devices on dabus
	List<Double> deviceErrorsRio = Arrays.asList(new Double[3]); // 4 can devices on rio can

	InetAddress radio;
	InetAddress limelight;
	InetAddress orangepi1;
	InetAddress orangepi2;

	SerialPort serialMXP = new SerialPort(115200, SerialPort.Port.kMXP);

	public void run() {
		serialMXP.disableTermination();
		try {
			radio = InetAddress.getByName(Constants.Network.radio);
			limelight = InetAddress.getByName(Constants.Network.limelight);
			orangepi1 = InetAddress.getByName(Constants.Network.orangepi1);
			orangepi2 = InetAddress.getByName(Constants.Network.orangepi2);
		} catch (Exception e) {
			System.out.println(e);
		}

		while (true) {
			loopCounter++;
			try {
				if (!radio.isReachable(20)) {
					radioErrCount++;
					Logger.getInstance().recordOutput("Ethernet/Radio", false);
				} else {
					Logger.getInstance().recordOutput("Ethernet/Radio", true);
				}

				if (!limelight.isReachable(20)) {
					limelightErrCount++;
					Logger.getInstance().recordOutput("Ethernet/Limelight", false);
				} else {
					Logger.getInstance().recordOutput("Ethernet/Limelight", true);
				}

				if (!orangepi1.isReachable(20)) {
					orangepi1ErrCount++;
					Logger.getInstance().recordOutput("Ethernet/Orangepi1", false);
				} else {
					Logger.getInstance().recordOutput("Ethernet/Orangepi1", true);
				}

				if (!orangepi2.isReachable(20)) {
					orangepi2ErrCount++;
					Logger.getInstance().recordOutput("Ethernet/Orangepi2", false);
				} else {
					Logger.getInstance().recordOutput("Ethernet/Orangepi2", true);
				}

			} catch (Exception e) {
				System.out.println(e);
			}

			int i = 0;
			deviceErrorsDabus.set(1, RobotContainer.swerve.getSteerErrorConc(i));
			deviceErrorsDabus.set(2, RobotContainer.swerve.getAzimuthErrorConc(i));
			deviceErrorsDabus.set(3, RobotContainer.swerve.getDriveErrorConc(i));

			deviceErrorsDabus.set(4, RobotContainer.superstructure.getFollowerErrorConc());

			i = 2;
			deviceErrorsDabus.set(5, RobotContainer.swerve.getSteerErrorConc(i));
			deviceErrorsDabus.set(6, RobotContainer.swerve.getAzimuthErrorConc(i));

			deviceErrorsDabus.set(7, RobotContainer.superstructure.getDeployErrorConc());
			deviceErrorsDabus.set(8, RobotContainer.superstructure.getRollerErrorConc());

			deviceErrorsDabus.set(9, RobotContainer.swerve.getDriveErrorConc(i));

			i = 2;
			deviceErrorsDabus.set(10, RobotContainer.swerve.getSteerErrorConc(i));
			deviceErrorsDabus.set(11, RobotContainer.swerve.getAzimuthErrorConc(i));
			deviceErrorsDabus.set(12, RobotContainer.swerve.getDriveErrorConc(i));

			deviceErrorsDabus.set(13, RobotContainer.climber.getClimberErrorConc());

			deviceErrorsDabus.set(14, RobotContainer.superstructure.getLeaderErrorConc());

			i = 1;
			deviceErrorsDabus.set(15, RobotContainer.swerve.getSteerErrorConc(i));
			deviceErrorsDabus.set(16, RobotContainer.swerve.getAzimuthErrorConc(i));
			deviceErrorsDabus.set(17, RobotContainer.swerve.getDriveErrorConc(i));

			RobotContainer.swerve.resetError();

			System.out.println("Arm Motor: " + RobotContainer.superstructure.getArmErrorConc() * 100 + "%");
			System.out.println("Arm Encoder: " + RobotContainer.superstructure.getArmAzimuthErrorConc() * 100 + "%");
			RobotContainer.climber.resetError();

			System.out.println("----------- Ethernet Total Failed Packet Counts (20ms timeout) -----------");
			System.out.println("Radio: " + radioErrCount);
			System.out.println("Limelight: " + limelightErrCount);
			System.out.println("Orangepi1: " + orangepi1ErrCount);
			System.out.println("Orangepi2: " + orangepi2ErrCount);

			if (loopCounter % 1000 == 0) {
				CANStatus status = new CANStatus();
				CANJNI.getCANStatus(status);
				serialMXP.writeString("<" + "?" + "," + status.percentBusUtilization + "," + "OK" + "," + "?");
			}
		}
	}
}
