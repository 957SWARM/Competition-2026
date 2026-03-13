package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

@Logged
public class ShiftTracker {

    double shiftTime = 25;

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > FieldConstants.TRANSITION_END) {
            // Transition shift, hub is active.
            shiftTime = matchTime - FieldConstants.TRANSITION_END;
            return true;
        } else if (matchTime > FieldConstants.SHIFT_1_END) {
            // Shift 1
            shiftTime = matchTime - FieldConstants.SHIFT_1_END;
            return shift1Active;
        } else if (matchTime > FieldConstants.SHIFT_2_END) {
            // Shift 2
            shiftTime = matchTime - FieldConstants.SHIFT_2_END;
            return !shift1Active;
        } else if (matchTime > FieldConstants.SHIFT_3_END) {
            // Shift 3
            shiftTime = matchTime - FieldConstants.SHIFT_3_END;
            return shift1Active;
        } else if (matchTime > FieldConstants.SHIFT_4_END) {
            // Shift 4
            shiftTime = matchTime - FieldConstants.SHIFT_4_END;
            return !shift1Active;
        } else {
            // End game, hub always active.
            shiftTime = matchTime;
            return true;
        }
    }

    public String shiftStatus(){
        if (shiftTime > FieldConstants.SHIFT_ALERT){
            return FieldConstants.DEFAULT_COLOR;
        }
        else if (shiftTime > FieldConstants.SHIFT_WARNING){
            return FieldConstants.ALERT_COLOR;
        }
        else {
            return FieldConstants.WARNING_COLOR;
        }
    }

}
