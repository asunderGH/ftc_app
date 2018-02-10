package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Nithya on 12/6/2017.
 */

@Autonomous( name = "Autonomous R1", group = "Copmetition")
public class NS_OpMode_Autonomous_R1 extends NS_OpMode_Autonomous {
    @Override
    public void DriveAutonomous() {
        if (opModeIsActive()) KnockJewel(Jewel.BLUE);

        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, -33, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, 110.0);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 13.5, 0.0);
    }
}
