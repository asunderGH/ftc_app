package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Four Wheel Drive Test", group = "Validation")
public class NS_Validation_FourWheelDrive extends LinearOpMode {
    DcMotor frontRightMotor = null;
    DcMotor frontLeftMotor = null;
    DcMotor backRightMotor = null;
    DcMotor backLeftMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double rightMotorPower = Range.clip(drive-turn, -1.0, 1.0);
            double leftMotorPower = Range.clip(drive+turn, -1.0, 1.0);
            SetDriveMotorPower(leftMotorPower,rightMotorPower);
        }

    }

    private void SetDriveMotorPower(double leftSideMotorPower, double rightSideMotorPower){

        frontRightMotor.setPower(rightSideMotorPower);
        frontLeftMotor.setPower(leftSideMotorPower);
        backRightMotor.setPower(rightSideMotorPower);
        backLeftMotor.setPower(leftSideMotorPower);
    }
}
