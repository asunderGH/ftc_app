package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private double drivePower = 0.0;
    private double driveDistance = 0.0;

    private DcMotor armElevationMotor = null;
    private double armMotorPower = 0.0;

    private Servo clawLeftServo = null;
    private Servo clawRightServo = null;

    private final double encoderTotalPulses = 1440;
    private final double turnsShaftToWheel = 2;
    private final double wheelCircumference = 4 * Math.PI;
    private final double encoderPulsesPerInch = encoderTotalPulses
                                                * turnsShaftToWheel
                                                / wheelCircumference;

    private ModernRoboticsI2cGyro driveGyro = null;




    NS_Robot_GoldenGears(HardwareMap hm) {
        hardwareMap = hm;

        driveLeftMotor = hardwareMap.dcMotor.get("driveLeftMotor");
        driveRightMotor = hardwareMap.dcMotor.get("driveRightMotor");
        driveRightMotor.setDirection(DcMotor.Direction.REVERSE);

        armElevationMotor = hardwareMap.dcMotor.get("armElevationMotor");

        clawLeftServo = hardwareMap.servo.get("clawLeftServo");
        clawRightServo = hardwareMap.servo.get("clawRightServo");
        clawRightServo.setDirection(Servo.Direction.REVERSE);

        driveGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("driveGyro");
        driveGyro.calibrate();

        this.Reset();
    }

    public void Reset() {
        this.ResetDrive();
        this.ResetEncoders();
        this.ResetServo();
    }

    public void ResetDrive() {
        driveLeftMotor.setPower(0);
        driveRightMotor.setPower(0);
        armElevationMotor.setPower(0);
    }

    public void ResetEncoders () {
        driveLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armElevationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ResetServo() {
        clawRightServo.setPosition(1.0);
        clawLeftServo.setPosition(1.0);
    }

    public void ResetGyro(){
        driveGyro.resetZAxisIntegrator();
    }

    public boolean IsReady() {
        return !driveGyro.isCalibrating();
    }

    public boolean IsDriving() {
        /* This is important to have the && logic instead of || logic
         * for the determination that desired travel was achieved
         * for the given angle.
         */
        return (driveLeftMotor.isBusy() && driveRightMotor.isBusy());
    }

    public void DriveTank(double leftPower, double rightPower) {
        // Normalize speeds if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0)
        {
            leftPower /= max;
            rightPower /= max;
        }

        driveLeftMotor.setPower(leftPower);
        driveRightMotor.setPower(rightPower);
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

    public void OnDriveDistance(double distance, double power) {
        driveLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Determine new target position, and pass to motor controller
         * for all given angles. Alternate is to calculate the target
         * position based on the drive angle and set them as position.
         */
        int moveCounts = (int)(distance * encoderPulsesPerInch);
        int newLeftTarget = driveLeftMotor.getCurrentPosition() + moveCounts;
        int newRightTarget = driveRightMotor.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        driveLeftMotor.setTargetPosition(newLeftTarget);
        driveRightMotor.setTargetPosition(newRightTarget);

        driveLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        driveDistance = distance;
        drivePower = Range.clip(Math.abs(power), 0.0, 1.0);
        this.DriveTank(drivePower, drivePower);
    }

    public void OnDriveAngle(double angle, double driveCoeff) {
        double  max;
        double  error;
        double  steer;
        double  leftPower;
        double  rightPower;

        // adjust relative speed based on heading error.
        error = getError(angle);
        steer = getSteer(error, driveCoeff);

        // if driving in reverse, the motor correction also needs to be reversed
        if (driveDistance < 0)
            steer *= -1.0;

        leftPower = drivePower - steer;
        rightPower = drivePower + steer;

        this.DriveTank(leftPower, rightPower);

        // Display drive status for the driver.
        /*
        telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
        telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
        telemetry.update();
        */
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param turnCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean OnDriveTurn(double power, double angle, double turnCoeff, double threshold) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftPower;
        double rightPower;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftPower  = 0.0;
            rightPower = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, turnCoeff);
            rightPower  = power * steer;
            leftPower   = -rightPower;
        }

        // Send desired speeds to motors.
        this.DriveTank(leftPower, rightPower);

        // Display it for the driver.
        /*
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        */
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - driveGyro.getIntegratedZValue();
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
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
