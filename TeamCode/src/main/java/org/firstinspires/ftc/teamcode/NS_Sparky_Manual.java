package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Sparky Manual Mode", group = "Competition")

public class NS_Sparky_Manual extends NS_Robot_Sparky {
    public final class PowerRegulator {
        static final double FULL = 1.0;
        static final double THREEFOURTH = 0.75;
        static final double ONEHALF = 0.5;
        static final double ONETHIRD = 1.0/3.0;
        static final double ONEFOURTH = 0.25;
        static final double ONEFIFTH = 0.20;
        static final double ONETENTH = 0.10;
    }

    double driveRegulator = PowerRegulator.FULL;
    double cargoLiftRegulator = PowerRegulator.FULL;
    double bucketElevationRegulator = PowerRegulator.ONEFOURTH;
    double collectorRegulator = PowerRegulator.ONEHALF;

    @Override
    public void runOpMode() throws InterruptedException {
        SparkyInitialize();

        waitForStart();

        SparkyStart();
        WaitWhileBusy();

        while (opModeIsActive()) {

            /* Configure the regulation for all motors */
            if (gamepad1.a == true) driveRegulator = PowerRegulator.ONEHALF;
            if (gamepad1.b == true) driveRegulator = PowerRegulator.FULL;
            //Regulating Speed For Cargo Lift
            if (gamepad1.x == true) bucketElevationRegulator = PowerRegulator.ONEFOURTH;
            if (gamepad1.y == true) bucketElevationRegulator = PowerRegulator.ONETHIRD;



            /* Program Driving Per User Input */
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double rightMotorPower = Range.clip(drive-turn, -1.0, 1.0);
            double leftMotorPower = Range.clip(drive+turn, -1.0, 1.0);
            RCDrive(leftMotorPower * driveRegulator, rightMotorPower * driveRegulator);

            if (gamepad1.right_bumper == true) {
                ActuateAutonomousServo(teamMarkerServoPositionDump);
            }
            else if (gamepad1.left_bumper == true) {
                ActuateAutonomousServo(teamMarkerServoPositionRest);
            }

                        /* Cargo Lift Actuation */
            // TBD: Revisit this code after setting up limits for user control
            boolean cargoLiftBumperPressed = false;
            if (gamepad2.left_bumper == true)
            {
                cargoLiftBumperPressed = true;

                //Raising The Cargo Lift
                double cargoLiftPower = -gamepad2.left_stick_y;
                ElevateCargoLift(cargoLiftPower * cargoLiftRegulator);
            }
            else
            {
                if (cargoLiftBumperPressed == true)
                {
                    ElevateCargoLift(0);
                }
                cargoLiftBumperPressed = false;

                /* Move Cargo Lift To Preprogrammed Positions */
                if (gamepad2.dpad_up == true) {
                    SetCargoLiftPositionByEncoder(cargoLiftEncoderPulsesHighPosition, cargoLiftRegulator);
                }
                else if(gamepad2.dpad_down == true) {
                    SetCargoLiftPositionByEncoder(cargoLiftEncoderPulsesLowPosition, cargoLiftRegulator);
                }
                else if(gamepad2.dpad_left == true) {
                    SetCargoLiftPositionByEncoder(cargoLiftEncoderPulsesOneThirdPosition, cargoLiftRegulator);
                }
                else if(gamepad2.dpad_right == true) {
                    SetCargoLiftPositionByEncoder(cargoLiftEncoderPulsesTwoThirdsPosition, cargoLiftRegulator);
                }
            }

            int moveCounts = 0;


            /* Bucket Elevation Control */
            boolean bucketElevatorBumperPressed = false;
            if (gamepad2.right_bumper == true) {
                bucketElevatorBumperPressed = true;

                double bucketLiftPower = gamepad2.right_stick_y;
                ElevateCargoBucket(bucketLiftPower * bucketElevationRegulator);
            }
            else {
                if (bucketElevatorBumperPressed == true) { ElevateCargoBucket(0); }
                bucketElevatorBumperPressed = false;

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

        SparkyStop();
        wait(1000);

    }
}
