package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Sparky Autonomous Crater", group = " Autonomous")
public class NS_Sparky_Autonomous_Crater extends NS_Sparky_Autonomous {
    @Override
    public void PreProgrammedPlay() throws InterruptedException {

        GyroTurn(1.0, 45);
        //WaitWhileDriving();

        GyroDrive(1.0, 45, 0.0);
        //WaitWhileDriving();

        GyroTurn(1.0, 120);
        //WaitWhileDriving();

        GyroDrive(1.0, 48, 0.0);
        //WaitWhileDriving();

        AutonomousDepotClaim();

        GyroTurn(1.0, -60);
        //WaitWhileDriving();

        GyroDrive(1.0, 58, 0.0);
        //WaitWhileDriving();

        ActuateAutonomousServo(teamMarkerServoPositionCrater);

        /*GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, 0.0);
        WaitWhileBusy();

        //Needs Actual Distance To Be Measured
        GyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 32, 0.0);
        WaitWhileBusy();*/
    }
}
