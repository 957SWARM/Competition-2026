package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TargetingConstants;

public class TargetingHelper {
    
    static double[] voltage = 
    {5.50, 5.50, 5.50, 5.50, 6.25, 6.25, 6.3, 6.50, 6.50, 7.00, 7.00}; 
    static double[] angle = 
    {0.05, 0.05, 0.20, 0.25, 0.30, 0.35, 0.45, 0.40, 0.45, 0.50, 0.50};
    static double[] distancePoints = 
    {0.00, 1.52, 2.09, 2.55, 3.07, 3.55, 3.72, 4.07, 4.50, 5.00, 300};

    public static String alliance = "Red";

    public static double calculateWeight(double distance){
   
        double closest = distancePoints[getNearestPoint(distance)];

        double weight = (distance - closest) / (distancePoints[getNearestPoint(distance) + 1] - closest);
        return weight;
    }

    public static double getExpectedHoodPosition(double distance){
        double w = calculateWeight(distance);
        int closest = getNearestPoint(distance);

        System.out.println(closest);
        
        double targetAngle = (angle[closest] * (1 - w)) + (angle[closest + 1] * w);
        return targetAngle;
    }

    public static double getExpectedShooterVoltage(double distance){
        double w = calculateWeight(distance);
        int closest = getNearestPoint(distance);
        
        double targetAngle = (voltage[closest] * (1 - w)) + (voltage[closest + 1] * w);
        return targetAngle;
    }

    /* 
    public static double getExpectedShooterVoltage(double distance) {
        double expectedVoltage = Math.sqrt((distance * 9.805967)/Math.sin(2*getExpectedHoodPosition(distance))) * Constants.HoodConstants.VELOCITY_TO_VOLTS;
        return expectedVoltage;
    }*/

    public static int getNearestPoint(double distance){

        if(distance < distancePoints[0])
            return 0;

        for(int p = 1; p < distancePoints.length - 2; p++){
            if(distancePoints[p] > distance)
                return p-1;
        }
        return 0;
    }

    public static Rotation2d getAngleToGoalPose(Pose2d robotPose, Pose2d goalPose){

        Pose2d poseDifference = robotPose.relativeTo(goalPose);
        Rotation2d outputAngle = new Rotation2d(poseDifference.getX(), poseDifference.getY());
        
        //System.out.println(outputAngle);

        // REMOVE PLUS !!!!!!
        return outputAngle;
    }

    public static double getDistanceToGoalPose(Pose2d robotPose, Pose2d goalPose){
        
        Pose2d poseDifference = robotPose.relativeTo(goalPose);
        double relativeX = poseDifference.getX();
        double relativeY = poseDifference.getY();
        double distance = Math.hypot(relativeX, relativeY);

        return distance;
    }

    //Prevents near-point correction
    public static double boundedPLoop(double min, double max, double setpoint, double kP, double measurement, double deadband){
        
        double error = setpoint - measurement;
        double output;

        if(Math.abs(error) < deadband) 
            return 0;

        output = error * kP;

        if(output < min && output > 0)
            return min;
        if(output > -min && output < 0)
            return -min;
        if(output > max)
            return max;
        if(output < -max)
            return -max;

        return output;
    }

    //Wrap-around code sponsered by ChatGPT
    public static double getRotationSpeed(Pose2d goalPose, Pose2d botPose){
        double botHeading = botPose.getRotation().getDegrees();

        double targetHeading = TargetingHelper.getAngleToGoalPose(botPose, goalPose).getDegrees();
                
        double error = targetHeading - botHeading;
        error = MathUtil.inputModulus(error, -180.0, 180.0);

        double angularSpeed = TargetingHelper.boundedPLoop(
            TargetingConstants.MIN,
            TargetingConstants.MAX,
            0,                     // setpoint is 0 error
            TargetingConstants.KP,
            error,                 // measurement is error
            TargetingConstants.DEADBAND
        );

        return -angularSpeed;
    }

    public static Pose2d getHubPose2d(){
        Pose2d targetHub = new Pose2d(FieldConstants.RED_HUB_LOCATION.getX(), FieldConstants.RED_HUB_LOCATION.getY(), Rotation2d.fromDegrees(0));
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            targetHub = new Pose2d(FieldConstants.BLUE_HUB_LOCATION.getX(), FieldConstants.BLUE_HUB_LOCATION.getY(), Rotation2d.fromDegrees(0));
            alliance = "Blue";
        }
        return targetHub;
    }

    public static Pose2d getPassPose2d(Pose2d robotPose){
        Pose2d targetPass = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        if(alliance == "Blue"){    
            if(robotPose.getY() < 4){
                targetPass = FieldConstants.BLUE_BOTTOM_PASSZONE;
            } else {
                targetPass = FieldConstants.BLUE_TOP_PASSZONE;
            }   
        } else {
            if(robotPose.getY() < 4){
                targetPass = FieldConstants.RED_BOTTOM_PASSZONE;
            } else {
                targetPass = FieldConstants.RED_TOP_PASSZONE;
            }
        
        }
        return targetPass;
    }

}
