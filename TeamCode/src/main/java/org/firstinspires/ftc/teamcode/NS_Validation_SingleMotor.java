package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Motor Test", group = "Validation")
public class NS_Validation_SingleMotor extends LinearOpMode {

    private DcMotor singleSpinMotor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        singleSpinMotor = hardwareMap.dcMotor.get("singlesMotor");

        waitForStart();

        while (opModeIsActive()) {
            double spinPower = gamepad1.left_stick_y;
            SpinMotor(spinPower);
        }

    }

    private void SpinMotor(double spinPower) {
        singleSpinMotor.setPower(spinPower);
    }
}
