package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Sparky Cargo Hold Test", group = "Tests")
public class NS_Sparky_Manual extends NS_Robot_Sparky {
    private final class PowerRegulator {
        static final double FULL = 1.0;
        static final double THREEFOURTH = 0.75;
        static final double HALF = 0.5;
        static final double ONETHIRD = 1/3;
        static final double FOURTH = 0.25;
        static final double TENTH = 0.10;
    }

    double latchRegulator = PowerRegulator.ONETHIRD;
    double cargoLiftRegulator = PowerRegulator.FULL;
    double bucketRegulator = PowerRegulator.HALF;
    double collectorRegulator = PowerRegulator.HALF;
    double driveRegulator = PowerRegulator.ONETHIRD;



    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();

        waitForStart();

        Start();

        while (opModeIsActive()) {

            if (gamepad1.a == true) {
                driveRegulator = PowerRegulator.HALF;
            }
            else if (gamepad1.b == true) {
                driveRegulator = PowerRegulator.FULL;
            }

            if (gamepad1.x == true) {
                latchRegulator = PowerRegulator.ONETHIRD;
            }
            else if (gamepad1.y == true) {
                latchRegulator = PowerRegulator.THREEFOURTH;
            }


            //Regulating Speed For The Collection Elevation Motor
            if (gamepad2.a == true) {
                collectorRegulator = PowerRegulator.FOURTH;
            }
            else if (gamepad2.b == true) {
                collectorRegulator = PowerRegulator.HALF;
            }

            //Regulating Speed For Cargo Lift
            if (gamepad2.x == true) {
                bucketRegulator = PowerRegulator.FOURTH;
            }
            else if (gamepad2.y == true) {
                bucketRegulator = PowerRegulator.HALF;
            }


            //Regulating Speed For The Bucket Elevation Motor
            if (gamepad2.dpad_up == true) {
                cargoLiftRegulator = PowerRegulator.HALF;
            }
            else if (gamepad2.dpad_down == true) {
                cargoLiftRegulator = PowerRegulator.FOURTH;
            }

            //if (gamepad1.right_trigger > 0.5)
            LatchArmPower(gamepad1.right_trigger); // * latchRegulator);
            //else if (gamepad1.left_trigger > 0.5) {
            LatchArmPower(-gamepad1.left_trigger); // * latchRegulator);


            //Makeing Cargo Lift Move To Preprogrammed Positions
            if (gamepad2.right_bumper == true) {
                SetCargoLiftPositionByEncoder(encoderPulsesHighPosition, cargoLiftRegulator);
            }
            else if(gamepad2.left_bumper == true) {
                SetCargoLiftPositionByEncoder(encoderPulsesLowPosition, cargoLiftRegulator);
            }

            //Driving
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double rightMotorPower = Range.clip(drive-turn, -1.0, 1.0);
            double leftMotorPower = Range.clip(drive+turn, -1.0, 1.0);
            RCDrive(leftMotorPower * driveRegulator, rightMotorPower * driveRegulator);

            // Moving The Bucket Up Or Down
            double bucketElevationPower = -gamepad2.left_stick_y;
            BucketElevation(bucketElevationPower * bucketRegulator);

            // Moving The Collector Up Or Down
            double collectorElevationPower = gamepad2.right_stick_y;
            ElevateCollector(collectorElevationPower * collectorRegulator);

            //if (gamepad2.right_trigger > 0.1) {
                RotateCollector(gamepad2.right_trigger);

            //else if (gamepad2.left_trigger > 0.1) {
                RotateCollector(-gamepad2.left_trigger);


        }

        Stop();
        wait(1000);

    }
}
