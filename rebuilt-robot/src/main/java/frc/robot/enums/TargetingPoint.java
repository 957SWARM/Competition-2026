package frc.robot.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public enum TargetingPoint {

    BLUE_HUB(new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0))),
    RED_HUB(new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0))),
    BLUE_BOTTOM(new Pose2d(1.9, 1.5, new Rotation2d())),
    BLUE_TOP(new Pose2d(1.9, 6.6, new Rotation2d())),
    RED_BOTTOM(new Pose2d(14.5, 1.5, new Rotation2d())),
    RED_TOP(new Pose2d(14.5, 6.6, new Rotation2d())),
    BLUE_CLIMB_RIGHT(new Pose2d(2.31, 3.86, new Rotation2d())),
    BLUE_CLIMB_LEFT(new Pose2d()),
    RED_CLIMB_RIGHT(new Pose2d()),
    RED_CLIMB_LEFT(new Pose2d());

    private final Pose2d point;

    private TargetingPoint(Pose2d point) {
        this.point = point;
    }

    public Pose2d getPoint(){
        return this.point;
    }

    public double getDistanceToPoint(Pose2d botPose){
        Pose2d poseDifference = botPose.relativeTo(this.point);
        double relativeX = poseDifference.getX();
        double relativeY = poseDifference.getY();
        double distance = Math.hypot(relativeX, relativeY);

        return distance;
    }

    public Rotation2d getAngleFromPoint(Pose2d botPose){
        Pose2d poseDifference = botPose.relativeTo(this.point);
        Rotation2d outputAngle = new Rotation2d(poseDifference.getX(), poseDifference.getY());

        return outputAngle;
    }

    public static TargetingPoint getPointToTarget(DriverStation.Alliance alliance, Pose2d botPose){

        TargetingPoint targetPass = BLUE_HUB;
        if(alliance == DriverStation.Alliance.Blue){    
            if(botPose.getX() > 4){
                if(botPose.getY() < 4){
                    targetPass = BLUE_BOTTOM;
                } else {
                    targetPass = BLUE_TOP;
                }
            } else {
                targetPass = BLUE_HUB;
            }  
        } else {
            if(botPose.getX() < 11){
                if(botPose.getY() < 4){
                    targetPass = RED_BOTTOM;
                } else {
                    targetPass = RED_TOP;
                }
            } else {
                targetPass = RED_HUB;
            }
        }
        return targetPass;

    }
    
    public static TargetingPoint getClimbPoint(DriverStation.Alliance alliance, Pose2d botPose){
        return BLUE_CLIMB_RIGHT;
    }
}
