package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Sparky Autonomous Depot", group = " Autonomous")
public class NS_Sparky_Autonomous_Depot extends NS_Sparky_Autonomous {
    @Override
    public void AutonomousPreProgrammedMode() throws InterruptedException {
        AutonomousStart();

        gyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, 0.0);
        WaitForSparky();

        // Disabled until there is a solution to reset an elevated bucket in the manual mode
        //SetCargoBucketPositionByEncoder(bucketElevationCraterPosition, NS_Sparky_Manual.PowerRegulator.ONETENTH);
        //WaitForSparky();

        gyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, GetCurrentAngle()-45);
        WaitForSparky();

        //Needs Actual Distance To Be Measured
        gyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 40, GetCurrentAngle());
        WaitForSparky();

        gyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, GetCurrentAngle()-90);
        WaitForSparky();

        gyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 30, GetCurrentAngle());
        WaitForSparky();

    }
}
