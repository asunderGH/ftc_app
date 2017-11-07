package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nithilan on 10/29/2017.
 * Tank Drive opmode for Golden Gear robot.
 */
@TeleOp(name = "NS: Tank Drive", group = "Training")
public class NS_OpMode_TankDrive extends OpMode {
    NS_Robot_GoldenGears GGRobot = null;
    boolean driveMode = false;

    @Override
    public void init() {
        GGRobot = new NS_Robot_GoldenGears(hardwareMap);
        telemetry.addData("Status", "Robot Initalized");
        telemetry.addData("Drive Mode", "Initalized to RC Drive");
    }

    @Override
    public void start() {
        GGRobot.Reset();
        telemetry.addData("Status", "Robot Started");
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;

        if (gamepad1.right_bumper == true) {
            driveMode = false;
            telemetry.addData("Drive Mode", "Changed to RC Drive");
        }
        else if (gamepad1.left_bumper == true) {
            driveMode = true;
            telemetry.addData("Drive Mode", "Changed to Tank Drive");
        }

        if (driveMode == false) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        }
        else {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        }
        GGRobot.DriveRobot(leftPower, rightPower);

        if (gamepad1.dpad_up == true) {
            GGRobot.RotateArm(1.0);
        }
        else if (gamepad1.dpad_down == true) {
            GGRobot.RotateArm(-1.0);
        }
        else {
            GGRobot.RotateArm(0.0);
        }

        if (gamepad1.dpad_left == true) {
            GGRobot.ActuateClaw(true);
        }
        else if (gamepad1.dpad_right == true) {
            GGRobot.ActuateClaw(false);
        }
    }

    @Override
    public void stop() {
        GGRobot.Reset();
        telemetry.addData("Status", "This Is AWESOME!");
    }
}