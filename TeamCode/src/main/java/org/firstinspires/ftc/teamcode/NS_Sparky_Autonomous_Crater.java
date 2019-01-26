package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Sparky Autonomous Crater", group = " Autonomous")
public class NS_Sparky_Autonomous_Crater extends NS_Sparky_Autonomous {
    @Override
    public void AutonomousPreProgrammedMode() throws InterruptedException {
        AutonomousStart();

        gyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, 0.0);
        WaitForSparky();

        // Disabled until there is a solution to reset an elevated bucket in the manual mode
        //SetCargoBucketPositionByEncoder(bucketElevationCraterPosition, NS_Sparky_Manual.PowerRegulator.ONETENTH);
        //WaitForSparky();

        //Needs Actual Distance To Be Measured
        gyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 32, 0.0);
        WaitForSparky();
    }
}
