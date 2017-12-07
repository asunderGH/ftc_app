package org.firstinspires.ftc.teamcode;

/**
 * Created by Nithya on 12/6/2017.
 */

public class NS_OpMode_Autonomous_R1 extends NS_OpMode_Autonomous {
    @Override
    public void DriveAutonomous() {
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, -24, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, 45.0);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, -13.42, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, 45.0);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 12.0, 0.0);
    }
}
