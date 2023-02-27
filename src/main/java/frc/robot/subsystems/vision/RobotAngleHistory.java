package frc.robot.subsystems.vision;

import java.util.TreeMap;
import java.util.Map.Entry;

public class RobotAngleHistory {

    private static TreeMap<Double, Double> timeInterpolatingTreeMap = new TreeMap<Double, Double>();

    public static void update(double timestamp, double robotAngle) {
        timeInterpolatingTreeMap.put(timestamp, robotAngle);

        if (timeInterpolatingTreeMap.size() > 50.0) {
            timeInterpolatingTreeMap.pollFirstEntry();
        }
    }

    public static double get(double timestamp) {
        Entry<Double, Double> ceil = timeInterpolatingTreeMap.ceilingEntry(timestamp);
        Entry<Double, Double> floor = timeInterpolatingTreeMap.floorEntry(timestamp);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        double t = (timestamp - floor.getKey()) / (ceil.getKey() - floor.getKey()); // Value between 0 and 1 signifying the distance the interpolated point is between the ceiling and floor keys
        return lerp(ceil.getValue(), floor.getValue(), t);
    }

    public static void clear() {
        timeInterpolatingTreeMap.clear();
    }

    private static double lerp(double y1, double y2, double t) {
        return y1 + (y2 - y1) + t;
        // y1 + (y2 - y1) * t
    }
    
}
