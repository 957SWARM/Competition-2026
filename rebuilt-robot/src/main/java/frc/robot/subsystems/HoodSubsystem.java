package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase{
    
    CANcoder hoodEncoder = new CANcoder(HoodConstants.ENCODER_ID);

    TalonSRX hood = new TalonSRX(HoodConstants.HOOD_ID);

    boolean zeroed = false;

    public HoodSubsystem(){
        hood.configRemoteFeedbackFilter(HoodConstants.ENCODER_ID, RemoteSensorSource.CANCoder, 0);
        hood.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 0);
    }

    public boolean isStopped(){
        return Math.abs(hoodEncoder.getVelocity().getValueAsDouble()) < 0.08;
    }

    public boolean hasBeenZeroed(){
        return zeroed;
    }
  
    public void resetHood(){
        hoodEncoder.setPosition(0);
    }

    public Command zeroEncoder(){
        return this.run(() -> 
            hood.set(ControlMode.PercentOutput, HoodConstants.ZEROING_SPEED)
        );
    }

    public Command driveHood(DoubleSupplier supplier){
        return this.run(() ->
            hood.set(ControlMode.MotionMagic, supplier.getAsDouble())
        );
    }



}
