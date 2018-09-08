package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class NS_Robot_SwivelTest extends LinearOpMode {
    private DcMotor leftSwivelMotor = null;
    private DcMotor rightSwivelMotor = null;
    private double leftSwivelPower = 0.0;
    private double rightSwivelPower = 0.0;

    private Servo leftSwivelServo = null;
    private Servo rightSwivelServo = null;
    private double servoPosition = 0.0;

    public void Reset() {
        this.ResetDriveMotors();
        this.ResetDriveServos();
    }

    public void ResetDriveMotors() {
        leftSwivelMotor.setPower(0.0);
        rightSwivelMotor.setPower(0.0);
    }

    public void ResetDriveServos() {
        leftSwivelServo.setPosition(0.0);
        rightSwivelServo.setPosition(0.0);
    }
}
