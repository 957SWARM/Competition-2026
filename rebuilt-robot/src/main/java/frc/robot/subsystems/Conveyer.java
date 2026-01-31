package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyer extends SubsystemBase{

    boolean isRunning = false;

    TalonFX motor = new TalonFX(0);

    public Command runConveyerForwards(){
        return this.run(() ->
        );
    }

    public Command stopConveyer(){
        return this.run(() ->
        );
    }

    public BooleanSupplier isRunningSupplier(){
        return () -> motor.get() != 0;
    }

}
