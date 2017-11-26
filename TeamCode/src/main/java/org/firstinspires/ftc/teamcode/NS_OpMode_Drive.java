package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Nithilan on 10/29/2017.
 * Tank Drive opmode for Golden Gear robot.
 */
@TeleOp(name = "NS: OpMode Drive", group = "Competition")
public class NS_OpMode_Drive extends OpMode {
    private final class Regulator {
        static final double FULL = 1.0;
        static final double HALF = 0.5;
        static final double FOURTH = 0.25;
        static final double TENTH = 0.10;
    }
    private enum DriveMode {
        RC_DRIVE, TANK_DRIVE
    }
    NS_Robot_GoldenGears GGRobot = null;
    DriveMode driveMode = DriveMode.RC_DRIVE;
    double driveRegulator = Regulator.FOURTH;
    double armRegulator = Regulator.TENTH;

    @Override
    public void init() {
        GGRobot = new NS_Robot_GoldenGears(hardwareMap);
        telemetry.addData("Status", "Robot Initalized");
        telemetry.addData("Drive Mode", "Initalized to RC Drive");
        telemetry.addData("Drive Speed", "Initialized to FOURTH");
    }

    @Override
    public void start() {
        GGRobot.Reset();
        telemetry.addData("Status", "Robot Started");
    }

    @Override
    public void loop() {
        if (gamepad1.right_stick_button == true) {
            driveMode = DriveMode.RC_DRIVE;
            telemetry.addData("Drive Mode", "Changed to RC Drive");
        }
        else if (gamepad1.left_stick_button == true) {
            driveMode = DriveMode.TANK_DRIVE;
            telemetry.addData("Drive Mode", "Changed to Tank Drive");
        }

        if (gamepad1.left_bumper == true) {
            driveRegulator = Regulator.FOURTH;
            telemetry.addData("Drive Speed", "FOURTH");
        }
        else if (gamepad1.right_bumper == true) {
            driveRegulator = Regulator.FULL;
            telemetry.addData("Drive Speed", "FULL");
        }

        double leftPower = 0.0;
        double rightPower = 0.0;
        if (driveMode == DriveMode.RC_DRIVE) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        }
        else {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        }
        GGRobot.DriveRobot(leftPower * driveRegulator,
                            rightPower * driveRegulator);

        double armPower = 0.0;
        if (gamepad1.dpad_up == true) {
            armPower = 1.0;
        }
        else if (gamepad1.dpad_down == true) {
            armPower = -1.0;
        }
        else {
            armPower = 0.0;
        }
        GGRobot.RotateArm(armPower * armRegulator);

        double advance = 0.005;
        if (gamepad1.dpad_left == true) {
            GGRobot.ActuateClaw(advance);
        }
        else if (gamepad1.dpad_right == true) {
            GGRobot.ActuateClaw(-advance);
        }
        
    }

    @Override
    public void stop() {
        GGRobot.Reset();
        telemetry.addData("Status", "This Is AWESOME!");
    }
}