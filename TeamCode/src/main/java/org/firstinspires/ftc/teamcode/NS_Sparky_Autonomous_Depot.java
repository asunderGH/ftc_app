package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Sparky Autonomous Depot", group = " Autonomous")
public class NS_Sparky_Autonomous_Depot extends NS_Sparky_Autonomous {
    @Override
    public void PreProgrammedPlay() throws InterruptedException {
        GyroTurn(1.0, 45);
        //WaitWhileDriving();

        GyroDrive(1.0, 45, 0.0);
        //WaitWhileDriving();

        GyroTurn(1.0, -35);
        //WaitWhileDriving();

        GyroDrive(1.0, 30, 0.0);
        //WaitWhileDriving();

        AutonomousDepotClaim();

        GyroTurn(1.0, 115);
        //WaitWhileDriving();

        GyroDrive(1.0, 59, 0.0);
        //WaitWhileDriving();

        ActuateAutonomousServo(teamMarkerServoPositionCrater);

        /*GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, 0.0);
        WaitWhileBusy();

        GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, GetCurrentAngle()-45);
        WaitWhileBusy();

        //Needs Actual Distance To Be Measured
        GyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 40, GetCurrentAngle());
        WaitWhileBusy();

        GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, GetCurrentAngle()-90);
        WaitWhileBusy();

        GyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 30, GetCurrentAngle());
        WaitWhileBusy();*/

    }
}
