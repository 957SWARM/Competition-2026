package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.Constants.PowerConstants;
@Logged
public class KickerSubsystem extends SubsystemBase {

    TalonFX kicker = new TalonFX(ConveyerConstants.KICKER_ID);

    VelocityVoltage request;

    public KickerSubsystem(){

        kicker.getConfigurator().apply(ConveyerConstants.kickerConfig);
        kicker.getConfigurator().apply(PowerConstants.mid_low);

        request = new VelocityVoltage(1).withSlot(0);
    }

    public Command runKicker(){
        return this.run(() ->
            kicker.setControl(request.withVelocity(ConveyerConstants.KICK_VELOCITY))
        );
    }

    public Command maxKicker(){
        return this.run(() ->
            kicker.setControl(request.withVelocity(ConveyerConstants.MAX_KICK_VELOCITY))
        );
    }

    public Command stopKicker(){
        return this.run(() ->
            kicker.setVoltage(0)
        );
    }

    public Command idleKicker(){
        return this.run(() ->
            kicker.setControl(request.withVelocity(ConveyerConstants.IDLE_KICK_VELOCITY))
        );
    }

    public BooleanSupplier isKickerRunning(){
        return () -> kicker.get() != 0;
    }

    public BooleanSupplier isKickerFeeding(){
        return () -> kicker.get() > 0;
    }

    public double getKickerVelocity(){
        return kicker.getVelocity().getValueAsDouble();
    }

}
