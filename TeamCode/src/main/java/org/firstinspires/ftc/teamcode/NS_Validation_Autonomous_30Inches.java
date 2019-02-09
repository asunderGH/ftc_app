package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous 30 Inches Test", group = " Validation")

public class NS_Validation_Autonomous_30Inches extends NS_Sparky_Autonomous {
    public void PreProgrammedPlay() throws InterruptedException {
        GyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, 30, GetCurrentAngle());
   }

}
