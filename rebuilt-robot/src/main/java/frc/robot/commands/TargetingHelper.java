package frc.robot.commands;

public class TargetingHelper {
    
    static double[] velocity = {0, 0, 0, 0, 0};
    static double[] angle = {0, 0, 0, 0, 0};
    static double[] distancePoints = {0, 0, 0, 0, 0};

    public static double calculateWeight(double distance){
   
        double closest = distancePoints[getNearestPoint(distance)];

        double weight = (distance - closest) / (distancePoints[getNearestPoint(distance) + 1] - closest);
        return weight;
    }

    public static double getExpectedHoodPosition(double currentDistance){
        double w = calculateWeight(currentDistance);
        int closest = getNearestPoint(currentDistance);
        
        double targetAngle = (angle[closest] * (1 - w)) + (angle[closest + 1] * w);
        return targetAngle;
    }

    public static double getExpectedShooterVoltage(double distance) {
        double expectedVoltage = Math.sqrt((distance * 9.805967)/Math.sin(2*getExpectedHoodPosition(distance))) * 1;
        return expectedVoltage;
    }

    public static int getNearestPoint(double distance){

        if(distance < distancePoints[0])
            return -1;

        for(int p = 0; p < distancePoints.length - 2; p++){
            if(distancePoints[p] < distance)
                return p;
        }
        return -1;
    }

}
