package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Turn Test", group = "Validation")
public class NS_Validation_Autonomous_Turn extends NS_Robot_Sparky {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialze_Sparky();
        waitForStart();
        Start_Sparky();

        //SetCargoLiftPositionByEncoder(encoderPulsesHighPosition, 1.0);
        //while (opModeIsActive() && IsClimbing());
        gyroTurn(0.4, 20);
        //wait(1000);
        //while ((opModeIsActive() && IsDriving()));
        //SetCargoLiftPositionByEncoder(encoderPulsesLowPosition, 1.0);

        Stop_Sparky();
    }
}
