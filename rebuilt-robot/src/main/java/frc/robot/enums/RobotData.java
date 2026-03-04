package frc.robot.enums;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class RobotData {
    
    public static RobotData instance;

    public static double distanceToTarget = 0;
    public static Rotation2d angleToTarget = new Rotation2d();
    public static TargetingPoint visiontarget = TargetingPoint.BLUE_HUB;
    public static TargetingPoint climbTarget = TargetingPoint.BLUE_CLIMB_RIGHT;
    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double thetaVelocity = 0;
    public static Pose2d botPose = new Pose2d();

    private RobotData(){

    }

    public static RobotData getInstance(){
        if(instance == null){
            instance = new RobotData();
        }
        return instance;
    }

    public void updateTargetingInfo(TargetingPoint visionPoint, TargetingPoint climbPoint, Pose2d botPose){
        visiontarget = visionPoint;
        angleToTarget = visiontarget.getAngleFromPoint(botPose);
        distanceToTarget = visiontarget.getDistanceToPoint(botPose);
        climbTarget = climbPoint;
    }

    public void updateDriveInfo(SwerveDriveState states){
        xVelocity = states.Speeds.vxMetersPerSecond;
        yVelocity = states.Speeds.vyMetersPerSecond;
        thetaVelocity = states.Speeds.omegaRadiansPerSecond;
        botPose = states.Pose;
    }
    

}  