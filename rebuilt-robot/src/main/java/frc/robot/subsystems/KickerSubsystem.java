package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyerConstants;

public class KickerSubsystem extends SubsystemBase {

    TalonFX kicker = new TalonFX(ConveyerConstants.KICKER_ID);

    double kickerVoltage;

    public KickerSubsystem(){

        kicker.getConfigurator().apply(ConveyerConstants.kickerConfig);
    }

    public Command runKicker(){
        return this.run(() ->
            kicker.setVoltage(kickerVoltage)
        );
    }

    public Command stopKicker(){
        return this.run(() ->
            kicker.setVoltage(0)
        );
    }

    public BooleanSupplier isKickerRunning(){
        return () -> kicker.get() != 0;
    }
}
