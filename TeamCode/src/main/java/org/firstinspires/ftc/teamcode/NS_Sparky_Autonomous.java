package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Sparky Autonomous Mode", group = "Competition")
public class NS_Sparky_Autonomous extends NS_Robot_Sparky {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialze_Sparky();

        waitForStart();

        Start_Sparky();
        WaitForSparky();

        SetCargoLiftPositionByEncoder(encoderPulsesHighPosition, 1.0);
        //while (opModeIsActive() && IsClimbing());
        WaitForSparky();
        gyroTurn(0.2, 20);
        //wait(1000);
        //while ((opModeIsActive() && IsDriving()));
        WaitForSparky();
        SetCargoLiftPositionByEncoder(encoderPulsesLowPosition, 1.0);
        WaitForSparky();

        Stop_Sparky();
    }
}
