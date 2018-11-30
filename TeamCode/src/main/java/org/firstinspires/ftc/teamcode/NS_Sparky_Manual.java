package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sparky Cargo Hold Test", group = "Tests")
public class NS_Sparky_Manual extends NS_Robot_Sparky {
    private final class SpeedRegulator {
        static final double FULL = 1.0;
        static final double HALF = 0.5;
        static final double FOURTH = 0.25;
        static final double TENTH = 0.10;
    }

    double driveRegulator = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {

     waitForStart();
     Initialization();
     while (opModeIsActive()) {

         if (gamepad2.x == true) {
             driveRegulator = SpeedRegulator.FOURTH;
         }
         else if (gamepad2.y == true) {
             driveRegulator = SpeedRegulator.HALF;
         }

         if (gamepad2.right_bumper == true) {
             SetCargoLiftPositionByEncoder(encoderPulsesHighPosition, driveRegulator);
         }
         else if(gamepad2.left_bumper == true) {
             SetCargoLiftPositionByEncoder(encoderPulsesLowPosition, driveRegulator);
         }
         else if (gamepad2.a == true) {
             SetCargoLiftPositionByEncoder(encoderPulses100, driveRegulator);
         }
         else if (gamepad2.b == true) {
             SetCargoLiftPositionByEncoder(encoderPulses200, driveRegulator);
         }
     }

    }
}
