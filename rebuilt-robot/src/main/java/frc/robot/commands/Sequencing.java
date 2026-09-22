package frc.robot.commands;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.enums.TargetingPoint;
import frc.robot.SwarmDriveController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.enums.TargetingPoint;

@Logged
public class Sequencing {
    
    private static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private static SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public static Command zeroHood(HoodSubsystem hood){
        return new WaitCommand(0.5).andThen(hood.zeroEncoder()
                    .withTimeout(HoodConstants.ZEROING_TIME)
                    .andThen(
                        hood.zeroEncoder()
                        .until(() -> hood.isStopped())).andThen(() -> hood.homeHood()));
    }

    public static Command agitate(PivotSubsystem pivot){
        return pivot.agitate().withTimeout(PivotConstants.AGITATION_TIME)
            .andThen(pivot.deploy().withTimeout(PivotConstants.AGITATION_TIME));
    }

    public static Command feedFuel(ConveyerSubsystem conveyer, KickerSubsystem kicker){
        return conveyer.runConveyerForwards().alongWith(kicker.runKicker());
    }

    /*public static Command fire(ShooterSubsystem shooter, HoodSubsystem hood, CommandSwerveDrivetrain drivetrain){
        return hood.setHoodPosition(TargetingHelper.getExpectedHoodPosition(TargetingPoint.getDist(drivetrain.getCurrentPose(), TargetingPoint.getPointToTarget(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue), drivetrain.getCurrentPose()))));
    }*/

    private static Command facePoint(CommandSwerveDrivetrain drivetrain, SwarmDriveController xbox) {
        return Commands.run(() -> drivetrain.setControl(
            drive.withVelocityX(-xbox.getYLimitedInput() * DriveConstants.MAX_STRAFE_SHOOT_SPEED)
                .withVelocityY(-xbox.getXLimitedInput() * DriveConstants.MAX_STRAFE_SHOOT_SPEED)
                .withRotationalRate(TargetingHelper.getRotationSpeed())
        ), drivetrain).until(TargetingHelper::isAlignedToTarget);
    }

    public static Command shoot(HoodSubsystem hood, PivotSubsystem pivot, ConveyerSubsystem conveyer, KickerSubsystem kicker, ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain, SwarmDriveController xbox){
        return facePoint(drivetrain, xbox).alongWith(feedFuel(conveyer, kicker));
    }
}

