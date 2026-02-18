package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.Constants.PowerConstants;
@Logged
public class ConveyerSubsystem extends SubsystemBase{

    boolean isRunning = false;

    TalonFX conveyer = new TalonFX(ConveyerConstants.CONVEYER_ID);

    public ConveyerSubsystem(){

        conveyer.getConfigurator().apply(ConveyerConstants.conveyerConfig);
        conveyer.getConfigurator().apply(PowerConstants.mid_low);

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

    public Command runConveyerBackwards(){
        return this.run(() -> 
            conveyer.setVoltage(-ConveyerConstants.FEED_VOLTAGE)
        );
    }

}
