package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TargetingConstants;
import frc.robot.enums.RobotData;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetingHelper {
    
    static double[] voltage = 
    {2750, 2750, 2750, 2750, 3125, 3125, 3150, 3250, 3300, 3500, 4000, 3500}; 
    static double[] angle = 
    {0.05, 0.05, 0.20, 0.25, 0.30, 0.35, 0.45, 0.40, 0.57, 0.60, 0.75, 0.60};
    static double[] distancePoints = 
    {0.00, 1.52, 2.09, 2.55, 3.07, 3.55, 3.72, 4.07, 4.50, 5.00, 16, 300};

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
        double botHeading = RobotData.botPose.getRotation().getDegrees();

        double targetHeading = RobotData.angleToTarget.getDegrees();
         
                
        double error = targetHeading - botHeading + getAlphaAngleOffset(RobotData.yVelocity).getDegrees();
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


}
