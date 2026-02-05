package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TargetingConstants;

public class TargetingHelper {
    
    static double[] velocity = {0, 0, 0, 0, 0};
    static double[] angle = {0, 0, 0, 0, 0};
    static double[] distancePoints = {0, 0, 0, 0, 0};

    public static double calculateWeight(double distance){
   
        double closest = distancePoints[getNearestPoint(distance)];

        double weight = (distance - closest) / (distancePoints[getNearestPoint(distance) + 1] - closest);
        return weight;
    }

    public static double getExpectedHoodPosition(double distance){
        double w = calculateWeight(distance);
        int closest = getNearestPoint(distance);
        
        double targetAngle = (angle[closest] * (1 - w)) + (angle[closest + 1] * w);
        return targetAngle;
    }

    public static double getExpectedShooterVoltage(double distance) {
        double expectedVoltage = Math.sqrt((distance * 9.805967)/Math.sin(2*getExpectedHoodPosition(distance))) * Constants.HoodConstants.VELOCITY_TO_VOLTS;
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

    public static Rotation2d getAngleToGoalPose(Pose2d robotPose, Pose2d goalPose){

        Pose2d poseDifference = robotPose.relativeTo(goalPose);
        Rotation2d outputAngle = new Rotation2d(poseDifference.getX(), poseDifference.getY());

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

    public static double getRotationSpeed(Pose2d goalPose, Pose2d botPose){
        double botHeading = botPose.getRotation().getDegrees();

        double targetHeading = TargetingHelper.getAngleToGoalPose(botPose, goalPose).getRotations();

        double measurementOne = botHeading;
        double measurementTwo = botHeading;

        if(Math.abs(targetHeading) < 0)
            measurementOne += 360;
         
        double measurement;

        if(targetHeading - measurementOne > measurementTwo - targetHeading){
            measurement = measurementTwo;
        } else {
            measurement = measurementOne;
        }

        double angularSpeed = TargetingHelper.boundedPLoop(TargetingConstants.MIN,
                                                            TargetingConstants.MAX,
                                                            targetHeading,
                                                            TargetingConstants.KP,
                                                            measurement,
                                                            TargetingConstants.DEADBAND);

        return angularSpeed;
    }

    public static Pose2d getHubPose2d(){
        Pose2d targetHub = new Pose2d(FieldConstants.RED_HUB_LOCATION.getX(), FieldConstants.RED_HUB_LOCATION.getY(), Rotation2d.fromDegrees(0));
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            targetHub = new Pose2d(FieldConstants.BLUE_HUB_LOCATION.getX(), FieldConstants.BLUE_HUB_LOCATION.getY(), Rotation2d.fromDegrees(0));
        }
        return targetHub;
    }

}
