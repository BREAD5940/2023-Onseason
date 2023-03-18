package frc.robot;

import java.net.InetAddress;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class EthernetLogger extends Thread {
    public int radioErrCount = 0;
    public int limelightErrCount = 0;
    public int orangepi1ErrCount  = 0;
    public int orangepi2ErrCount = 0;
  
    InetAddress radio;
    InetAddress limelight;
    InetAddress orangepi1;
    InetAddress orangepi2;

    public void run(){
        try{
            radio = InetAddress.getByName(Constants.Network.radio);
            limelight = InetAddress.getByName(Constants.Network.limelight);
            orangepi1 = InetAddress.getByName(Constants.Network.orangepi1);
            orangepi2 = InetAddress.getByName(Constants.Network.orangepi2);
            } catch (Exception e){
              System.out.println(e);
            }
        
        while(true){
            try{
                if(!radio.isReachable(20)){
                  radioErrCount++;
                  Logger.getInstance().recordOutput("Ethernet/Radio", false);
                } else {
                  Logger.getInstance().recordOutput("Ethernet/Radio", true);
                }
        
                if(!limelight.isReachable(20)){
                  limelightErrCount++;
                  Logger.getInstance().recordOutput("Ethernet/Limelight", false);
                } else {
                  Logger.getInstance().recordOutput("Ethernet/Limelight", true);
                }
        
                if(!orangepi1.isReachable(20)){
                  orangepi1ErrCount++;
                  Logger.getInstance().recordOutput("Ethernet/Orangepi1", false);
                } else {
                  Logger.getInstance().recordOutput("Ethernet/Orangepi1", true);
                }
        
                if(!orangepi2.isReachable(20)){
                  orangepi2ErrCount++;
                  Logger.getInstance().recordOutput("Ethernet/Orangepi2", false);
                } else {
                  Logger.getInstance().recordOutput("Ethernet/Orangepi2", true);
                }
        
              } catch(Exception e) {
                System.out.println(e);
              }
        }
     }
}
