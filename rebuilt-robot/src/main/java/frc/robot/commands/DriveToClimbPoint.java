package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.enums.RobotData;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToClimbPoint extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private double xvel, yvel, tvel = 0;

  private double maxAccleration = 0.1;
  private double maxAngular = 5;

  public DriveToClimbPoint(CommandSwerveDrivetrain drive) {
    drivetrain = drive;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    System.out.println("Super cool debug print for DriveToClimbPoint");
    updateSpeeds();
    System.out.println("Speeds are: " + xvel + ", " + yvel + ", and " + tvel);

      drivetrain.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(xvel * DriveConstants.MAX_SPEED)
            .withVelocityY(yvel * DriveConstants.MAX_SPEED)
            .withRotationalRate(tvel * DriveConstants.MAX_ANGULAR)
    );
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void updateSpeeds() {

    double newXSpeed = TargetingHelper.boundedPLoop(ClimberConstants.TRANSLATION_MIN, 
                                                    ClimberConstants.TRANSLATION_MAX, 
                                                    RobotData.climbTarget.getPoint().getX(), 
                                                    ClimberConstants.TRANSLATION_P, 
                                                    drivetrain.getCurrentPose().getX(), 
                                                    ClimberConstants.TRANSLATION_DEADBAND) * DriveConstants.MAX_SPEED;

    double newYSpeed = TargetingHelper.boundedPLoop(ClimberConstants.TRANSLATION_MIN, 
                                                    ClimberConstants.TRANSLATION_MAX, 
                                                    RobotData.climbTarget.getPoint().getY(), 
                                                    ClimberConstants.TRANSLATION_P, 
                                                    drivetrain.getCurrentPose().getY(), 
                                                    ClimberConstants.TRANSLATION_DEADBAND) * DriveConstants.MAX_SPEED;

    double newAngularSpeed = TargetingHelper.boundedPLoop(ClimberConstants.ROTATION_MIN,
                                                    ClimberConstants.ROTATION_MAX, 
                                                    RobotData.climbTarget.getPoint().getRotation().getDegrees(), 
                                                    ClimberConstants.ROTATION_P, 
                                                    drivetrain.getCurrentPose().getRotation().getDegrees(), 
                                                    ClimberConstants.ROTATION_DEADBAND) * DriveConstants.MAX_ANGULAR;                                                

    if(Math.abs(newXSpeed) < Math.abs(xvel) + maxAccleration){
      xvel = newXSpeed;
    } else {
      if(newXSpeed > 0){
        xvel += maxAccleration;
      } else {
        xvel -= maxAccleration;
      }
    }

    if(Math.abs(newYSpeed) < Math.abs(yvel) + maxAccleration){
      yvel = newYSpeed;
    } else {
      if(newYSpeed > 0){
        yvel += maxAccleration;
      } else {
        yvel -= maxAccleration;
      }
    }

    if(Math.abs(newAngularSpeed) < Math.abs(tvel) + maxAngular){
      tvel = newAngularSpeed;
    } else {
      if(newAngularSpeed > 0){
        tvel += maxAngular;
      } else {
        tvel -= maxAngular;
      }
    }


  }
}