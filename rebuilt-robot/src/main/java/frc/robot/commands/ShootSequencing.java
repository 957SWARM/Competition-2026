package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SwarmDriveController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public class ShootSequencing {

    private static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private static SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
    private static Command facePoint(CommandSwerveDrivetrain drivetrain, SwarmDriveController xbox) {
        return Commands.run(() -> drivetrain.setControl(
            drive.withVelocityX(-xbox.getYLimitedInput() * DriveConstants.MAX_STRAFE_SHOOT_SPEED)
                .withVelocityY(-xbox.getXLimitedInput() * DriveConstants.MAX_STRAFE_SHOOT_SPEED)
                .withRotationalRate(TargetingHelper.getRotationSpeed())
        ), drivetrain).until(TargetingHelper::isAlignedToTarget);
    }


    private static Command plant(CommandSwerveDrivetrain drivetrain) {
        return Commands.run(() -> drivetrain.setControl(brake), drivetrain);
    }

    public static Command shootSequence(
        CommandSwerveDrivetrain drivetrain,
        KickerSubsystem kicker,
        ConveyerSubsystem conveyor,
        RollerSubsystem roller)
    {
        return Commands.parallel(
                plant(drivetrain),
                shootNoLockSequence(kicker, conveyor, roller)
            );
    }

    public static Command shootNoLockSequence(
        KickerSubsystem kicker,
        ConveyerSubsystem conveyor,
        RollerSubsystem roller)
    {
        return Commands.parallel(
                kicker.runKicker(),
                roller.intakeCommand(),
                conveyor.runConveyerForwards()
            );
    }

    public static Command autoAlignAndShootSequence(
        CommandSwerveDrivetrain drivetrain,
        SwarmDriveController xbox,
        KickerSubsystem kicker,
        ConveyerSubsystem conveyor,
        RollerSubsystem roller) 
    {
        return Commands.sequence(
            facePoint(drivetrain, xbox),
            shootSequence(drivetrain, kicker, conveyor, roller)
        );
    }

    public static Command shootOnMoveSequence(
        CommandSwerveDrivetrain drivetrain,
        SwarmDriveController xbox,
        KickerSubsystem kicker,
        ConveyerSubsystem conveyor,
        RollerSubsystem roller) 
    {
        return Commands.parallel(
            facePoint(drivetrain, xbox),
            shootNoLockSequence(kicker, conveyor, roller)
        );
    }
}
