package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
    private ModernRoboticsI2cGyro gyroSensor = null;

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

        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();

        this.Reset();
    }

    public void Reset() {
        this.ResetDrive();

        driveLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armElevationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.ResetEncoders();

        clawRightServo.setPosition(1.0);
        clawLeftServo.setPosition(1.0);
    }

    public void ResetDrive() {
        driveLeftMotor.setPower(0);
        driveRightMotor.setPower(0);
        armElevationMotor.setPower(0);
    }

    public void ResetEncoders () {
        driveLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ResetGyro(){
        gyroSensor.resetZAxisIntegrator();
    }

    public boolean IsReady() {
        return !gyroSensor.isCalibrating();
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

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean OnGyroDrive(double power, double angle, double PCoeff, double threshold) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = power * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        driveLeftMotor.setPower(leftSpeed);
        driveRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyroSensor.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
