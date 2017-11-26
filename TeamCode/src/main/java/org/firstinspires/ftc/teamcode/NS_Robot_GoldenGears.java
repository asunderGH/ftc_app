package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nithya on 10/29/2017.
 * This class implements Golden Gears robot.
 * Any OpMode can use this class to control the hardware.
 */

public class NS_Robot_GoldenGears {
    // Hardware Map of robot
    private HardwareMap hardwareMap = null;

    private DcMotor driveLeftMotor = null;
    private DcMotor driveRightMotor = null;
    private DcMotor armElevationMotor = null;
    private Servo clawLeftServo = null;
    private Servo clawRightServo = null;

    private final double encoderTotalPulses = 1440;
    private final double turnsShaftToWheel = 2;
    private final double wheelCircumference = 4 * Math.PI;
    private final double encoderPulsesPerInch = encoderTotalPulses
                                                * turnsShaftToWheel
                                                / wheelCircumference;


    public NS_Robot_GoldenGears(HardwareMap hm) {
        hardwareMap = hm;

        driveLeftMotor = hardwareMap.dcMotor.get("driveLeftMotor");
        driveRightMotor = hardwareMap.dcMotor.get("driveRightMotor");
        driveRightMotor.setDirection(DcMotor.Direction.REVERSE);

        armElevationMotor = hardwareMap.dcMotor.get("armElevationMotor");

        clawLeftServo = hardwareMap.servo.get("clawLeftServo");
        clawRightServo = hardwareMap.servo.get("clawRightServo");
        clawRightServo.setDirection(Servo.Direction.REVERSE);

        this.Reset();
    }

    public void Reset() {
        driveLeftMotor.setPower(0);
        driveRightMotor.setPower(0);
        armElevationMotor.setPower(0);

        driveLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armElevationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.ResetEncoders();

        clawRightServo.setPosition(1.0);
        clawLeftServo.setPosition(1.0);
    }

    public void ResetEncoders () {
        driveLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void DriveRobot(double driveLeftPower, double driveRightPower) {
        driveLeftMotor.setPower(driveLeftPower);
        driveRightMotor.setPower(driveRightPower);
    }

    public void RotateArm(double armPower) {
        armElevationMotor.setPower(armPower);
    }

    public void ActuateClaw(double advance) {
        double position = clawLeftServo.getPosition() + advance;
        position = Range.clip(position, 0.0, 1.0);

        clawLeftServo.setPosition(position);
        clawRightServo.setPosition(position);
    }

}
