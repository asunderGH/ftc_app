package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous 45 Degrees Test", group = " Validation")

public class NS_Validation_Autonomous_45Degrees extends NS_Sparky_Autonomous {

   public void PreProgrammedPlay() throws InterruptedException {
        GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFOURTH, 45);
    }

}
