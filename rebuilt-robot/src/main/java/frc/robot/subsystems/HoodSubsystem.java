package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PowerConstants;
@Logged
public class HoodSubsystem extends SubsystemBase{
    
    CANcoder hoodEncoder = new CANcoder(HoodConstants.ENCODER_ID);

    TalonSRX hood = new TalonSRX(HoodConstants.HOOD_ID);

    boolean zeroed = false;

    PIDController loop = new PIDController(HoodConstants.HOOD_KP, 0, 0);

    public HoodSubsystem(){
        hood.configRemoteFeedbackFilter(HoodConstants.ENCODER_ID, RemoteSensorSource.CANCoder, 0);
        hood.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 0);

        hood.configPeakCurrentLimit(PowerConstants.HOOD_LIMIT);
        hood.setInverted(true);
    }

    public boolean isStopped(){
        return Math.abs(hoodEncoder.getVelocity().getValueAsDouble()) < 0.01;
    }

    public boolean hasBeenZeroed(){
        return zeroed;
    }
  
    public void homeHood(){
        hoodEncoder.setPosition(0);
    }

    public Command zeroEncoder(){
        return this.run(() -> hood.set(ControlMode.PercentOutput, HoodConstants.ZEROING_SPEED)
        );
    }

    public Command driveHood(DoubleSupplier positionSupplier, BooleanSupplier conveyerSupplier){
        return this.run(() -> {
            if(conveyerSupplier.getAsBoolean()){
                setHoodPosition(positionSupplier.getAsDouble());
            } else {
                setHoodPosition(positionSupplier.getAsDouble());
            }
        }
        );
    }

    public void setHoodPosition(double setpoint){
        double output = loop.calculate(hoodEncoder.getPosition().getValueAsDouble(), setpoint * HoodConstants.MAX_HOOD_POS);
        output = Math.min(output, 0.5);
        output = Math.max(output, -0.5);
        
        hood.set(ControlMode.PercentOutput, output);
    }



}
