
package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TargetingConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TargetingHelper {
    
    static double[] voltage = {5.5, 5.5, 5.5, 5.5, 6.25, 6.25, 6.5, 6.5, 7, 7}; 
    static double[] angle = {0.05, 0.05, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.5};
    static double[] distancePoints = {0, 1.52, 2.09, 2.55, 3.07, 3.55, 4.07, 4.5, 5.0, 300};

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

    public static Rotation2d getAngleToGoalPose(Rotation2d robotPose, Rotation2d goalPose){

        Rotation2d poseDifference = robotPose.minus(goalPose);
        //System.out.println(outputAngle);

        // REMOVE PLUS !!!!!!
        return poseDifference;
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
    public static double getRotationSpeed(Rotation2d goalPose, Rotation2d botPose){
        double botHeading = botPose.getDegrees();

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

    public static double getAdjustedRotationSpeed(Rotation2d correctedPose, Rotation2d botPose){
        double botHeading = botPose.getDegrees();

        double targetHeading = TargetingHelper.getAngleToGoalPose(botPose, correctedPose).getDegrees();
                
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

    public static Pose2d getPassTargetRotation2d(Pose2d robotPose){
        Pose2d targetPass = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        if(robotPose.getX() > 4.3){    
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
        } else {
            targetPass = getHubPose2d();
        }    
        return targetPass;
    }

    public static Rotation2d rotationWithOffSet(Pose2d currentPose, Pose2d targetPose, CommandXboxController xbox, double speedY, double distance){
        Pose2d futurePose2d = new Pose2d(currentPose.getX() + xbox.getRightY() * DriveConstants.M_SPEED,
                                         currentPose.getY() + xbox.getRightX() * DriveConstants.M_SPEED,
                                         Rotation2d.fromDegrees(currentPose.getRotation().getDegrees() + (xbox.getLeftX() * DriveConstants.M_ANGULAR_RATE))); 
        
        Rotation2d targetAngleCorrected = getAngleToGoalPose(currentPose.getRotation(),
        Rotation2d.fromDegrees(getAngleToGoalPose(currentPose.getRotation(), getPassTargetRotation2d(currentPose).getRotation()).getDegrees() - (TargetingConstants.DISTANCE_SCALAR * (1/(distance*distance)) * TargetingConstants.SPEED_SCALAR * speedY * TargetingConstants.CORRECTION_SCALAR * 0.5 * getAngleToGoalPose(currentPose.getRotation(), futurePose2d.getRotation()).getDegrees())));

        return targetAngleCorrected;
    }

}
