package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.PivotSubsystem;

@Logged
public class Sequencing {
    
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
}

