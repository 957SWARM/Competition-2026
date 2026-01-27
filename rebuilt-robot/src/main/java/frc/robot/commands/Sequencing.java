package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Sequencing {
    
    public static Command zeroHood(HoodSubsystem hood){
        return hood.zeroEncoder()
                    .withTimeout(HoodConstants.ZEROING_TIME)
                    .andThen(
                        hood.zeroEncoder()
                        .until(() -> hood.isStopped()));
    }

    public static Command shootToHub(RollerSubsystem roller, Conveyer conveyer){
        return roller.intakeCommand().alongWith(conveyer.runConveyerForwards());
    }

    public static Command shootFromNeutral(HoodSubsystem hood, ShooterSubsystem shooter, RollerSubsystem roller, Conveyer conveyer){
        return shootToHub(roller, conveyer)
            .alongWith(hood.driveHood(() -> Constants.HoodConstants.FROM_NEUTRAL_ANGLE))
            .alongWith(shooter.shoot(() -> Constants.ShooterConstants.FROM_NEUTRAL_VOLTAGE));
    }



}
