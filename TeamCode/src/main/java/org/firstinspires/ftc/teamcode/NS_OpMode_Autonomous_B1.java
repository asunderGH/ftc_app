package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Nithya on 12/6/2017.
 */

@Autonomous ( name = "Autonomous B1", group = "Copmetition")
public class NS_OpMode_Autonomous_B1 extends NS_OpMode_Autonomous {
    @Override
    public void DriveAutonomous() {
        if (opModeIsActive()) KnockJewel(Jewel.RED);

        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 29, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, 63);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 5.5, 0.0);
    }
}
