package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class NS_Robot_Sparky extends LinearOpMode {
    //DriveTrain Motors
    private DcMotor frontRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor backLeftMotor = null;
    //Cargo Lift Motor
    private DcMotor cargoLiftMotor = null;
    //Mineral Transporting Motor and Servos
    private DcMotor bucketElevationMotor = null;
    //Mineral Collection Motor
    private DcMotor mineralCollectionMotor = null;

    private double drivePower = 0.0;
    private double driveDistance = 0.0;

    // Encoder Calculations
    private final double encoderTotalPulses = 1440;
    private final double pinionCircumference = 19.085 * Math.PI;

    private final double cargoLiftMotorToPinionRatio = 2;
    private final double encoderPulsesPerMilimeter = encoderTotalPulses * cargoLiftMotorToPinionRatio / pinionCircumference;
    private final double cargoLiftBottomPosition = 0.0; //mm
    private final double cargoLiftTopPosition = 170; //mm, 6.7 inches
    private final double cargoLiftThresholdLimit = 2; //mm
    private final double cargoLiftLowPosition = cargoLiftBottomPosition; // + cargoLiftThresholdLimit;
    private final double cargoLiftHighPosition = cargoLiftTopPosition; // - cargoLiftThresholdLimit;
    public final int cargoLiftEncoderPulsesLowPosition = (int) (encoderPulsesPerMilimeter * cargoLiftLowPosition);
    public final int cargoLiftEncoderPulsesHighPosition = (int) (encoderPulsesPerMilimeter * cargoLiftHighPosition);
    public final int cargoLiftEncoderPulsesThresholdLimit = (int) (encoderPulsesPerMilimeter * cargoLiftThresholdLimit);
    public final int cargoLiftEncoderPulsesOneThirdPosition = cargoLiftEncoderPulsesHighPosition / 3;
    public final int cargoLiftEncoderPulsesTwoThirdsPosition = cargoLiftEncoderPulsesHighPosition * 2 / 3;

    private final double bucketElevationMotorToGearRatio = 2;
    private final double bucketElevationGearRotationsForCrater = 45.0 / 360.0; //45 Degrees from rest position
    private final double bucketElevationGearRotationsForTransport = 140.0 / 360.0;
    private final double bucketElevationGearRotationsForDumping = 190.0 / 360.0;
    private final double bucketElevationThresholdLimit = 3.0 / 360.0;
    public final int bucketElevationRestPosition = 5;
    public final int bucketElevationCraterPosition =
            (int) (encoderTotalPulses * bucketElevationMotorToGearRatio * bucketElevationGearRotationsForCrater);
    public final int bucketElevationTransportPosition =
            (int) (encoderTotalPulses * bucketElevationMotorToGearRatio * bucketElevationGearRotationsForTransport);
    public final int bucketElevationDumpPosition =
            (int) (encoderTotalPulses * bucketElevationGearRotationsForDumping * bucketElevationMotorToGearRatio);
    public final int bucketElevationEncoderPulsesHighPosition = (int) ( encoderPulsesPerMilimeter* bucketElevationDumpPosition);
    public final int bucketElevationEncoderPulsesLowPosition = (int) (encoderPulsesPerMilimeter* bucketElevationRestPosition);
    public final int bucketElevationEncoderPulsesThresholdLimit = (int) (encoderTotalPulses * bucketElevationThresholdLimit);


    public final int mineralCollectionElevatorUpPosition = 0;
    public final int mineralCollectionElevatorDownPosition = (int) (encoderTotalPulses * 0.6);

    private final double turnsShaftToWheel = 1;
    private final double wheelCircumference = 4.0 * Math.PI;
    private final double encoderPulsesPerInch = encoderTotalPulses
            * turnsShaftToWheel
            / wheelCircumference;


    // The IMU sensor object
    private BNO055IMU imu = null;
    private final int imuPollInterval = 50; // In milliseconds



    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.1;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.
    static final double     ARM_SPEED               = 0.2;

    // Gyro Drive
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    public void CreateHardwareMap() {
        //latchRotationMotor = hardwareMap.dcMotor.get("latchRotationMotor");
        //latchHookServo = hardwareMap.servo.get("latchHookServo");

        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        cargoLiftMotor = hardwareMap.dcMotor.get("cargoLiftMotor");

        bucketElevationMotor = hardwareMap.dcMotor.get("bucketElevationMotor");

        //mineralCollectionElevationMotor = hardwareMap.dcMotor.get("mineralCollectionElevationMotor");
        //mineralCollectionElevationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mineralCollectionMotor = hardwareMap.dcMotor.get("mineralCollectionMotor");

        // Map gyro sensor
        imu = hardwareMap.get(BNO055IMU.class, "imu18");
    }

    public void ResetDriveMotors() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    public void ResetDriveEncoders() {
        //Set Motors To Stop_Sparky and Reset Encoders
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cargoLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketElevationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION and have the motors RUN_USING_ENCODERS by default
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cargoLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketElevationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //mineralCollectionElevationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mineralCollectionElevationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ResetDrive() {
        ResetDriveMotors();
        ResetDriveEncoders();
    }

    public void ResetGyro() {
        //driveGyro.calibrate();
    }

    public void ResetIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "REVHubIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);
    }

    public void ResetAll() {
        ResetDrive();
        ResetIMU();
    }

    //Initialze_Sparky Method
    public void Initialze_Sparky() {
        telemetry.addData("Sparky: ", "Initializing ...");
        telemetry.update();

        CreateHardwareMap();
        ResetAll();
    }

    public void Start_Sparky() {
        telemetry.addData("Sparky: ", "Starting ...");
        telemetry.update();

        // When The Motor Recieves No Power, It Will Resist Any Outside Forces Placed Upon It
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cargoLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start IMU integration
        imu.startAccelerationIntegration(new Position(), new Velocity(), imuPollInterval);
    }

    public void Stop_Sparky() {
        telemetry.addData("Sparky: ", "Stopping ...");
        telemetry.update();

        SetCargoLiftPositionByEncoder(0, 0.25);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        cargoLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bucketElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean IsDriving() {
        /* This is important to have the && logic instead of || logic
         * for the determination that desired travel was achieved
         * for the given angle.
         */
        return (backRightMotor.isBusy() && backLeftMotor.isBusy());
    }

    public boolean IsClimbing() {
        return (cargoLiftMotor.isBusy());
    }

    public boolean IsBusy() {
        // return true if robot is idle
        return (IsDriving() || IsClimbing() || bucketElevationMotor.isBusy());
    }

    public void WaitWhileBusy() {
        while (IsBusy()) {
            sleep(50);
            idle();
        }
    }

    public void WaitForSparky() {
        while (!isStopRequested() && IsBusy()) {
            sleep(50);
            idle();
        }
    }


    //Driving Method
    public void RCDrive(double leftSideMotorPower, double rightSideMotorPower) {
        double max = Math.max(Math.abs(leftSideMotorPower), Math.abs(rightSideMotorPower));
        if (max > 1.0) {
            leftSideMotorPower /= max;
            rightSideMotorPower /= max;
        }

        backRightMotor.setPower(rightSideMotorPower);
        backLeftMotor.setPower(leftSideMotorPower);
        frontRightMotor.setPower(rightSideMotorPower);
        frontLeftMotor.setPower(leftSideMotorPower);
    }

    public void ElevateCargoLift(double cargoLiftPower) {
        cargoLiftPower = Range.clip(cargoLiftPower, -1.0, 1.0);
        double currentCargoLiftEncoderPosition = cargoLiftMotor.getCurrentPosition();
        if (cargoLiftPower > 0
                && currentCargoLiftEncoderPosition < (cargoLiftEncoderPulsesHighPosition - cargoLiftEncoderPulsesThresholdLimit))
        {
            cargoLiftMotor.setPower(cargoLiftPower);
        }
        else if (cargoLiftPower < 0)
                // && currentCargoLiftEncoderPosition > (cargoLiftEncoderPulsesLowPosition + cargoLiftEncoderPulsesThresholdLimit))
        {
            cargoLiftMotor.setPower(cargoLiftPower);
        }
        else {
            cargoLiftMotor.setPower(0);
        }
    }

    public void SetCargoLiftPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        telemetry.addData("SetCargoLiftMotorByEncoder: ", "%d", encoderTargetPosition); //String.valueOf(encoderTargetPosition));
        telemetry.update();

        cargoLiftMotor.setTargetPosition(encoderTargetPosition);
        cargoLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }


    public void ElevateCargoBucket(double cargoBucketLiftPower) {
        cargoBucketLiftPower = Range.clip(cargoBucketLiftPower, -1.0, 1.0);
        double currentBucketElevationEncoderPosition = bucketElevationMotor.getCurrentPosition();

        if (cargoBucketLiftPower > 0
                && currentBucketElevationEncoderPosition < (bucketElevationEncoderPulsesHighPosition - bucketElevationEncoderPulsesThresholdLimit))
        {
            bucketElevationMotor.setPower(cargoBucketLiftPower);
        }
        else if (cargoBucketLiftPower < 0
                && currentBucketElevationEncoderPosition > (bucketElevationEncoderPulsesLowPosition + bucketElevationEncoderPulsesThresholdLimit))
        {
            bucketElevationMotor.setPower(cargoBucketLiftPower);
        }
        else
        {
            bucketElevationMotor.setPower(0);
        }

    }

    public void SetCargoBucketPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        telemetry.addData("SetCargoBucketPositionByEncoder: ", "%d", encoderTargetPosition); //String.valueOf(encoderTargetPosition));
        telemetry.update();

        // Set Target and Turn On RUN_TO_POSITION
        bucketElevationMotor.setTargetPosition(encoderTargetPosition);
        bucketElevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        bucketElevationMotor.setPower(liftSpeed);
    }

    public void RotateMineralCollector(double collectionPower) {
        collectionPower = Range.clip(collectionPower, -1.0, 1.0);
        if (Math.abs(collectionPower) < 0.1) {
            collectionPower = 0;
        }
        mineralCollectionMotor.setPower(collectionPower);
    }



    public double GetCurrentAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (double) currentAngles.firstAngle;
    }

    /**
     * angularError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double angularError(double targetAngle) {
        // State used for updating telemetry
        Orientation orientationAngles;
        //Acceleration gravity;
        double angularError = 0.0;

        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        orientationAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //gravity  = imu.getGravity();


        // calculate error in -179 to +180 range  (
        angularError = targetAngle - orientationAngles.firstAngle;
        while (angularError > 180) angularError -= 360;
        while (angularError <= -180) angularError += 360;
        telemetry.addData("Target Angle: ", "%3.2f", targetAngle); //String.valueOf(encoderTargetPosition));
        telemetry.addData("Angular Error: ", "%3.2f", angularError); //String.valueOf(encoderTargetPosition));
        telemetry.update();
        return angularError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    private double angularSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void OnDriveDistance(double distance, double power) {
        /* Code to be removed
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        /* Determine new target position, and pass to motor controller
         * for all given angles. Alternate is to calculate the target
         * position based on the drive angle and set them as position.
         */
        int moveCounts = (int) (distance * encoderPulsesPerInch);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts;
        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
        int newBackRightTarget = backRightMotor.getCurrentPosition() + moveCounts;
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        telemetry.addData("Target",  "FrontRight:%d, FrontLeft:%d, BackRight:%d, BackLeft:%d",
                newFrontRightTarget, newFrontLeftTarget, newBackRightTarget, newBackLeftTarget);
        driveDistance = distance;
        drivePower = Range.clip(Math.abs(power), 0.0, 1.0);
        RCDrive(drivePower, drivePower);
    }

    public void OnDriveAngle(double distance, double angle, double driveCoeff) {
        double max;
        double error;
        double steer;
        double leftPower;
        double rightPower;

        // adjust relative speed based on heading error.
        error = angularError(angle);
        steer = angularSteer(error, driveCoeff);

        // if driving in reverse, the motor correction also needs to be reversed
        if (distance < 0)
            steer *= -1.0;

        // Validate the power applied to both sides
        rightPower = drivePower - steer;
        leftPower = drivePower + steer;

        RCDrive(leftPower, rightPower);

        // Display drive status for the driver.
        telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        telemetry.addData("Actual",  "%7d:%7d",
                backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        telemetry.addData("Speed",   "%5.2f:%5.2f",  leftPower, rightPower);
        telemetry.update();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param turnCoeff Proportional Gain coefficient
     * @return
     */
    public boolean OnDriveTurn(double power, double angle, double turnCoeff, double threshold) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftPower;
        double rightPower;

        // determine turn power based on +/- error
        error = angularError(angle);

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftPower = 0.0;
            rightPower = 0.0;
            onTarget = true;
        } else {
            steer = angularSteer(error, turnCoeff);
            // Ensure that the right power direction is applied
            rightPower = power * steer;
            leftPower = -rightPower;
        }

        // Send desired speeds to motors.
        RCDrive(leftPower, rightPower);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftPower, rightPower);

        return onTarget;
    }



    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {
        telemetry.addData("gyroDrive: ", "%5.2f", distance);
        telemetry.update();

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            OnDriveDistance(distance, speed);

            // keep looping while we are still active, and BOTH motors are running.
            /*
            while (opModeIsActive() && IsDriving()) {
                OnDriveAngle(distance, angle, P_DRIVE_COEFF);
            }
            */
            WaitWhileBusy();

            // Stop all motion;
            ResetDriveMotors();

            // Turn off RUN_TO_POSITION
            ResetDriveEncoders();
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn ( double speed, double angle ) throws InterruptedException {
        telemetry.addData("gyroTurn: ", "%5.2f", angle);
        telemetry.update();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !OnDriveTurn(speed, angle, P_TURN_COEFF, HEADING_THRESHOLD)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            OnDriveTurn(speed, angle, P_TURN_COEFF, HEADING_THRESHOLD);
            telemetry.update();
        }

        // Stop all motion;
        ResetDrive();
    }

}
