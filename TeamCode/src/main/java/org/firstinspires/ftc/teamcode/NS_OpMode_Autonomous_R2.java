package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Nithya on 12/6/2017.
 */

@Autonomous( name = "Autonomous R2", group = "Copmetition")
public class NS_OpMode_Autonomous_R2 extends NS_OpMode_Autonomous {
    @Override
    public void DriveAutonomous() {
        if (opModeIsActive()) KnockJewel(Jewel.BLUE);

        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, -30, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, 90.0);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, -26, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, 149);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 11.3, 0.0);
    }
}
