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

    PIDController loop = new PIDController(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD);

    public double incrementalHoodPos = .5;

    public HoodSubsystem(){
        hood.configRemoteFeedbackFilter(HoodConstants.ENCODER_ID, RemoteSensorSource.CANCoder, 0);
        hood.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 0);

        hood.configPeakCurrentLimit(PowerConstants.HOOD_LIMIT);
        hood.setInverted(true);
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
        double output = loop.calculate(getHoodPosition(), setpoint * HoodConstants.MAX_HOOD_POS);
        output = Math.min(output, HoodConstants.MAX_PERCENT_OUTPUT);
        output = Math.max(output, -HoodConstants.MAX_PERCENT_OUTPUT);
        
        hood.set(ControlMode.PercentOutput, output);
    }

    public double getHoodPosition(){
        return hoodEncoder.getPosition().getValueAsDouble();
    }

    public boolean isStopped(){
        return Math.abs(hoodEncoder.getVelocity().getValueAsDouble()) < 0.02;
    }

    public boolean hasBeenZeroed(){
        return zeroed;
    }

    //DEBUG COMMANDS

    public void increaseHoodPosition(){
        if(incrementalHoodPos < 1){
            incrementalHoodPos += 0.05;
        }
    }

    public void decreaseHoodPosition(){
        if (incrementalHoodPos > 0){
            incrementalHoodPos -= 0.05;
        }
    }

}
