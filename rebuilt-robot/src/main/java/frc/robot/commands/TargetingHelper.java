package frc.robot.commands;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.TargetingConstants;
import frc.robot.SwarmDriveController;
import frc.robot.enums.RobotData;


public class TargetingHelper {
    
    static double[] velocity = 
    {47.0, 47.0, 47.0, 49.0, 50.0, 53.0, 56.0, 57.0, 59.0, 61.0, 63.7, 65.7}; 
    static double[] angle = 
    {0.1, 0.1, 0.23, 0.28, 0.37, 0.415, 0.43, 0.45, 0.55, 0.55, 0.70, 0.70};
    static double[] distancePoints = 
    {0.00, 1.51, 2.02, 2.50, 3.05, 3.51, 3.70, 4.01, 4.50, 5.00, 16.0, 300};

    public static String alliance = "Red";

    public static double calculateWeight(double distance){
   
        double closest = distancePoints[getNearestPoint(distance)];

        double weight = (distance - closest) / (distancePoints[getNearestPoint(distance) + 1] - closest);
        return weight;
    }

    public static double getExpectedHoodPosition(double distance){
        double w = calculateWeight(distance);
        int closest = getNearestPoint(distance);

        //System.out.println(closest);
        
        double targetAngle = (angle[closest] * (1 - w)) + (angle[closest + 1] * w);
        return targetAngle;
    }

    public static double getExpectedShooterVelocity(double distance){
        double w = calculateWeight(distance);
        int closest = getNearestPoint(distance);
        
        double targetAngle = (velocity[closest] * (1 - w)) + (velocity[closest + 1] * w);
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

        for(int p = 1; p < distancePoints.length; p++){
            if(distancePoints[p] > distance)
                return p-1;
        }
        return 0;
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
    public static double getRotationSpeed(){
        double botHeading = RobotData.lookAheadPose.getRotation().getDegrees();

        double targetHeading = RobotData.angleToTarget.getDegrees();
         
                
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



    public static double getDriveSpeed(){
        double x = RobotData.xVelocity;
        double y = RobotData.yVelocity;
        
        return Math.sqrt((x*x) + (y*y));
    }

    public static Rotation2d getAlphaAngleOffset(double speedY){
        return Rotation2d.fromDegrees(TargetingConstants.kRa * speedY);
    }

    
    public static boolean isAlignedToTarget() {
        if(Math.abs(RobotData.angleToTarget.getDegrees() - RobotData.botPose.getRotation().getDegrees()) < TargetingConstants.TARGETING_DEADBAND 
        && Math.abs(RobotData.xVelocity) < 0.1 
        && Math.abs(RobotData.yVelocity) < 0.1
        && Math.abs(RobotData.thetaVelocity) < 0.04){
            return true;
        }
        else{
            return false;
        }
    }

    public static boolean isDriving(SwarmDriveController xbox){
        if(Math.abs(xbox.getXLimitedInput()) > 0.1 
        && Math.abs(xbox.getYLimitedInput()) > 0.1){
            return true;
        }
        else{
            return false;
        }
    }

}
