//0231 (DONT DELETE I NEED THIS COMMENT)

package frc.robot.FaultChecker;

import java.net.InetAddress;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;

import edu.wpi.first.wpilibj.SerialPort;

import org.littletonrobotics.junction.Logger;
import java.util.GregorianCalendar;

public class FaultCheckerOTHER {
    int loopCounter = 0;
    public int radioErrCount = 0;
    public int limelightErrCount = 0;
    public int orangepi1ErrCount = 0;
    public int orangepi2ErrCount = 0;
    private String[] swerveOrder = {"FL, FR, BL, BR"};

    double allowableCANerror = 0.01;

    String priorityCANerror = "OK";
    String priorityETHerror = "OK";
    String priorityTEMPerror = "OK";

    InetAddress radio;
    InetAddress limelight;
    InetAddress orangepi1;
    InetAddress orangepi2;

    SerialPort serialMXP = new SerialPort(115200, SerialPort.Port.kMXP);

    public FaultCheckerOTHER() {
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

    public void update() {
        System.out.println("---------- ERRORS ----------")
        priorityCANerror = "OK";
        priorityETHerror = "OK";

        loopCounter++;
        try {
            if (!limelight.isReachable(20)) {
                limelightErrCount++;
                Logger.getInstance().recordOutput("Ethernet/Limelight", false);
                System.out.println("Ethernet/Limelight: " + "false");
                priorityETHerror = "Limelight NC";
            } else {
                Logger.getInstance().recordOutput("Ethernet/Limelight", true);
                System.out.println("Ethernet/Limelight: " + "true");
            }

            if (!orangepi1.isReachable(20)) {
                orangepi1ErrCount++;
                Logger.getInstance().recordOutput("Ethernet/Orangepi1", false);
                System.out.println("Ethernet/Orangepi1: " + "false");
                priorityETHerror = "OPI1 NC";
            } else {
                Logger.getInstance().recordOutput("Ethernet/Orangepi1", true);
                System.out.println("Ethernet/Orangepi1: " + "true");
            }

            if (!orangepi2.isReachable(20)) {
                orangepi2ErrCount++;
                Logger.getInstance().recordOutput("Ethernet/Orangepi2", false);
                System.out.println("Ethernet/Orangepi2: " + "false");
                priorityETHerror = "OPI2 NC";
            } else {
                Logger.getInstance().recordOutput("Ethernet/Orangepi2", true);
                System.out.println("Ethernet/Orangepi2: " + "true");
            }

            long finish = 0;
            long start = new GregorianCalendar().getTimeInMillis();

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
        } catch (Exception e) {
            System.out.println(e);
        }

        if (RobotContainer.climber.getClimberErrorConc() > allowableCANerror) {
            priorityCANerror = "CLMB NC";
            System.out.println("ClimberErrorConc: " + "CLMB NC");
        }

        if (RobotContainer.superstructure.getDeployErrorConc() > allowableCANerror) {
            priorityCANerror = "INTK DPLY NC";
            System.out.println("DeployErrorConc: " + "INTK DPLY NC");
        }
        if (RobotContainer.superstructure.getRollerErrorConc() > allowableCANerror) {
            priorityCANerror = "INTK RLLR NC";
            System.out.println("RollerErrorConc: " + "INTK RLLR NC");
        }

        if (RobotContainer.superstructure.getFollowerErrorConc() > allowableCANerror) {
            priorityCANerror = "ELEV FLLWR NC";
            System.out.println("FollowerErrorConc: " + "ELEV FLLWR NC");
        }
        if (RobotContainer.superstructure.getLeaderErrorConc() > allowableCANerror) {
            priorityCANerror = "ELEV LDR NC";
            System.out.println("LeaderErrorConc: " + "ELEV LDR NC");
        }

        if (RobotContainer.superstructure.getArmAzimuthErrorConc() > allowableCANerror) {
            priorityCANerror = "ARM ENC NC";
            System.out.println("ArmAzimuthErrorConc: " + "ARM ENC NC");
        }
        if (RobotContainer.superstructure.getArmErrorConc() > allowableCANerror) {
            priorityCANerror = "ARM NC";
            System.out.println("ArmErrorConc: " + "ARM NC");
        }

        if (RobotContainer.superstructure.getEndEffectorErrorConc() > allowableCANerror) {
            priorityCANerror = "END EFFCTR NC";
            System.out.println("EndEffectorErrorConc: " + "END EFFCTR NC");
        }

        // loops through all the swerve
        for (int i = 0; i < 4; i++) { 
            if (RobotContainer.swerve.getDriveErrorConc(i) > 0.01) {
                priorityCANerror = "MOD " + i + " DRV NC";
                System.out.println(swerveOrder + " DriveErrorConc: STR NC")
            }
            if (RobotContainer.swerve.getSteerErrorConc(i) > 0.01) {
                priorityCANerror = "MOD " + i + " STR NC";
                System.out.println(swerveOrder + " SteerErrorConc: STR NC")
            }
            if (RobotContainer.swerve.getAzimuthErrorConc(i) > 0.01) {
                priorityCANerror = "MOD " + i + " ENC NC";
                System.out.println(swerveOrder + " AzimuthErrorConc: STR NC")
            }
        }

        // System.out.println("----------- Ethernet Total Failed Packet Counts (20ms
        // timeout) -----------");
        // System.out.println("Radio: " + radioErrCount);
        // System.out.println("Limelight: " + limelightErrCount);
        // System.out.println("Orangepi1: " + orangepi1ErrCount);
        // System.out.println("Orangepi2: " + orangepi2ErrCount);

        /*if (loopCounter % 100 == 0) {
            CANStatus status = new CANStatus();
            CANJNI.getCANStatus(status);
            // serialMXP.writeString("<0,0,0,0>");
            serialMXP.writeString("<" + "?," + ((int) (status.percentBusUtilization * 100)) + "," + priorityCANerror
                    + "_" + priorityETHerror + "," + "?>");
        }*/
        /*try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/


        System.out.print("---------- END OF ERRORS ----------");
    }
}
