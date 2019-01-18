package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Sparky Manual Mode", group = "Competition")

public class NS_Sparky_Manual extends NS_Robot_Sparky {
    private final class PowerRegulator {
        static final double FULL = 1.0;
        static final double THREEFOURTH = 0.75;
        static final double HALF = 0.5;
        static final double ONETHIRD = 1.0/3.0;
        static final double ONEFOURTH = 0.25;
        static final double TENTH = 0.10;
    }

    double driveRegulator = PowerRegulator.FULL;
    double cargoLiftRegulator = PowerRegulator.FULL;
    double bucketElevationRegulator = PowerRegulator.ONEFOURTH;
    double collectorRegulator = PowerRegulator.HALF;


    @Override
    public void runOpMode() throws InterruptedException {
        Initialze_Sparky();

        waitForStart();

        Start_Sparky();
        WaitForSparky();

        while (opModeIsActive()) {

            /* Configure the regulation for all motors */
            if (gamepad1.a == true) driveRegulator = PowerRegulator.HALF;
            if (gamepad1.b == true) driveRegulator = PowerRegulator.FULL;
            //Regulating Speed For Cargo Lift
            if (gamepad1.x == true) bucketElevationRegulator = PowerRegulator.ONEFOURTH;
            if (gamepad1.y == true) bucketElevationRegulator = PowerRegulator.ONETHIRD;


            /* Program Driving Per User Input */
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double rightMotorPower = Range.clip(drive-turn, -1.0, 1.0);
            double leftMotorPower = Range.clip(drive+turn, -1.0, 1.0);
            RCDrive(leftMotorPower * driveRegulator, rightMotorPower * driveRegulator);

            /* Cargo Lift Actuation */
            // TBD: Revisit this code after setting up limits for user control
            boolean cargoLiftUserOperation = false;
            if (cargoLiftUserOperation == true)
            {
                //Raising The Cargo Lift
                double cargoLiftPower = gamepad2.left_stick_y;
                ElevateCargoLift(cargoLiftPower);
            }
            else
            {
                /* Move Cargo Lift To Preprogrammed Positions */
                if (gamepad2.dpad_up == true) {
                    SetCargoLiftPositionByEncoder(encoderPulsesHighPosition, cargoLiftRegulator);
                }
                else if(gamepad2.dpad_down == true) {
                    SetCargoLiftPositionByEncoder(encoderPulsesLowPosition, cargoLiftRegulator);
                }
                else if(gamepad2.dpad_left == true) {
                    SetCargoLiftPositionByEncoder(encoderPulsesOneThirdPosition, cargoLiftRegulator);
                }
                else if(gamepad2.dpad_right == true) {
                    SetCargoLiftPositionByEncoder(encoderPulsesTwoThirdsPosition, cargoLiftRegulator);
                }
            }


            /* Bucket Elevation Control */
            boolean bucketLiftUserOperation = false;
            if (bucketLiftUserOperation == true) {
                double bucketLiftPower = gamepad2.right_stick_y;
                ElevateCargoBucket(bucketLiftPower * bucketElevationRegulator);
            }
            else {
                if (gamepad2.a == true) {
                    SetCargoBucketPositionByEncoder(bucketElevationRestPosition, bucketElevationRegulator);
                }
                else if (gamepad2.b == true) {
                    SetCargoBucketPositionByEncoder(bucketElevationCraterPosition, bucketElevationRegulator);
                }
                else if (gamepad2.x == true) {
                    SetCargoBucketPositionByEncoder(bucketElevationTransportPosition, bucketElevationRegulator);
                }
                else if (gamepad2.y == true) {
                    SetCargoBucketPositionByEncoder(bucketElevationDumpPosition, bucketElevationRegulator);
                }
            }

            if (gamepad2.right_trigger > 0.1) {
                RotateMineralCollector(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger > 0.1) {
                RotateMineralCollector(-gamepad2.left_trigger);
            }
            else {
                RotateMineralCollector(0);
            }

        }

        Stop_Sparky();
        wait(1000);

    }
}
