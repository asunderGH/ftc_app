package org.firstinspires.ftc.teamcode;

/**
 * Created by Nithya on 12/6/2017.
 */

public class NS_OpMode_Autonomous_B2 extends NS_OpMode_Autonomous {
    @Override
    public void DriveAutonomous() {
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 24, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, -20.0);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 4, 0.0);
    }
}
