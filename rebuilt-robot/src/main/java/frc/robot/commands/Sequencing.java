package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;

public class Sequencing {
    
    public Command zeroHood(HoodSubsystem hood){
        return hood.zeroEncoder()
                    .withTimeout(HoodConstants.ZEROING_TIME)
                    .andThen(
                        hood.zeroEncoder()
                        .until(() -> hood.isStopped()));
    }




}
