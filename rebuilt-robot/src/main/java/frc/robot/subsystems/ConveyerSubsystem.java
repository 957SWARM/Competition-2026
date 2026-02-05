package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyerConstants;

public class ConveyerSubsystem extends SubsystemBase{

    boolean isRunning = false;

    TalonFX conveyer = new TalonFX(ConveyerConstants.CONVEYER_ID);

    public ConveyerSubsystem(){

        conveyer.getConfigurator().apply(ConveyerConstants.conveyerConfig);

    }

    public Command runConveyerForwards(){
        return this.run(() ->
        conveyer.setVoltage(ConveyerConstants.FEED_VOLTAGE)
        );
    }

    public Command stopConveyer(){
        return this.run(() ->
        conveyer.setVoltage(0)
        );
    }

    public BooleanSupplier isConveyerRunningSupplier(){
        return () -> conveyer.get() != 0;
    }

}
