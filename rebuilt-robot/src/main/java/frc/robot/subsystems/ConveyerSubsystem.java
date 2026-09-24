package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
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
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ConveyerSubsystem(){

        conveyer.getConfigurator().apply(ConveyerConstants.conveyerConfig);
        conveyer.getConfigurator().apply(PowerConstants.mid_high);

    }

    public Command runConveyerForwards(){
        return this.runOnce(() ->
            conveyer.setControl(voltageRequest.withOutput(ConveyerConstants.FEED_VOLTAGE))//conveyer.setVoltage(ConveyerConstants.FEED_VOLTAGE)
        );
    }

    public Command stopConveyer(){
        return this.run(() ->
            conveyer.setControl(voltageRequest.withOutput(0))//conveyer.setVoltage(0)
        );
    }

    public Command pulseConveyor(){
        return runConveyerForwards().withTimeout(0.35).andThen(runConveyerBackwards().withTimeout(0.1));
    }

    public Command idleConveyer(){
        return this.run(() ->{
            conveyer.setVoltage(ConveyerConstants.IDLE_FEED_VOLTAGE);
            System.out.println("TESTING!!!");
        }
            
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
