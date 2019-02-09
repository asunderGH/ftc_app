package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Turn Test", group = "Validation")
public class NS_Validation_Autonomous_Turn extends NS_Robot_Sparky {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkyInitialize();
        waitForStart();
        SparkyStart();

        //SetCargoLiftPositionByEncoder(encoderPulsesHighPosition, 1.0);
        //while (opModeIsActive() && IsClimbing());
        GyroTurn(0.4, 20);
        //wait(1000);
        //while ((opModeIsActive() && IsDriving()));
        //SetCargoLiftPositionByEncoder(encoderPulsesLowPosition, 1.0);

        SparkyStop();
    }
}
