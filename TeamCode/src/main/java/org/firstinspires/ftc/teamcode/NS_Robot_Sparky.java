package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class NS_Robot_Sparky extends LinearOpMode {
    // Encoder Calculations
    private final double encoderTotalPulses = 1440;
    private final double ratioMotorToPinion = 2;
    private final double pinionCircumference = 19.085 * Math.PI;
    private final double encoderPulsesPerMilimeter = encoderTotalPulses * ratioMotorToPinion / pinionCircumference;
    private final double cargoLiftBottomPosition = 0.0; //mm
    private final double cargoLiftTopPosition = 280; //279.4; //mm, 11 inches
    private final double cargoLift100 = 100;
    private final double cargoLift200 = 200;
    private final double cargoLiftForestallLimit = 5; //mm
    private final double cargoLiftLowPosition = cargoLiftBottomPosition + cargoLiftForestallLimit;
    private final double cargoLiftHighPosition = cargoLiftTopPosition - cargoLiftForestallLimit;
    final int encoderPulsesLowPosition = (int) (encoderPulsesPerMilimeter * cargoLiftLowPosition);
    final int encoderPulsesHighPosition = (int) (encoderPulsesPerMilimeter * cargoLiftHighPosition);
    final int encoderPulses100 = (int) (encoderPulsesPerMilimeter * cargoLift100);
    final int encoderPulses200 = (int) (encoderPulsesPerMilimeter * cargoLift200);

    public final int bucketElevationUpPosition = (int) (encoderTotalPulses * 1.65);
    public final int bucketElevationDownPosition = 0;

    public final int mineralCollectionElevatorUpPosition = 0;
    public final int mineralCollectionElevatorDownPosition = (int) (encoderTotalPulses * 0.6);


    //DriveTrain Motors
    private DcMotor treadLeftMotor = null;
    private DcMotor treadRightMotor = null;

    //Cargo Lift Motor
    private DcMotor cargoLiftMotor = null;
    //Mineral Transporting Motor and Servos
    private DcMotor bucketElevationMotor = null;

    //Lander Latch Motor and Servo
    private DcMotor latchRotationMotor = null;

    private Servo latchHookServo = null;

    //Mineral Collection Motor
    private DcMotor mineralCollectionMotor = null;
    private DcMotor mineralCollectionElevationMotor = null;

    public void CreateHardwareMap() {
        latchRotationMotor = hardwareMap.dcMotor.get("latchRotationMotor");
        //latchHookServo = hardwareMap.servo.get("latchHookServo");

        treadLeftMotor = hardwareMap.dcMotor.get("treadLeftMotor");
        treadRightMotor = hardwareMap.dcMotor.get("treadRightMotor");
        treadRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        cargoLiftMotor = hardwareMap.dcMotor.get("cargoLiftMotor");

        bucketElevationMotor = hardwareMap.dcMotor.get("bucketElevationMotor");

        mineralCollectionElevationMotor = hardwareMap.dcMotor.get("mineralCollectionElevationMotor");
        mineralCollectionElevationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mineralCollectionMotor = hardwareMap.dcMotor.get("mineralCollectionMotor");
    }

    //Initialization Method
    public void Initialization() {
        CreateHardwareMap();
        latchRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cargoLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //mineralCollectionElevationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mineralCollectionElevationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Start() {
        // Set the cargo lift to starting position
        SetCargoLiftPositionByEncoder(encoderPulsesLowPosition,0.25);

        // When The Motor Recieves No Power, It Will Resist Any Outside Forces Placed Upon It
    }

    public void Stop() {
        SetCargoLiftPositionByEncoder(0, 0.25);

        mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void LatchArmPower(double latchMotorPower) {
        latchMotorPower = com.qualcomm.robotcore.util.Range.clip(latchMotorPower, -1.0, 1.0);
        if (Math.abs(latchMotorPower) < 0.1) {
            latchMotorPower = 0;
        }
        latchRotationMotor.setPower(latchMotorPower);
    }

    //Driving Method
    public void RCDrive(double leftPower, double rightPower) {
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        treadLeftMotor.setPower(leftPower);
        treadRightMotor.setPower(rightPower);
    }

    public void SetCargoLiftPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        cargoLiftMotor.setTargetPosition(encoderTargetPosition);
        cargoLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }

    public void BucketElevation(double bucketElevationPower) {
        bucketElevationPower = com.qualcomm.robotcore.util.Range.clip(bucketElevationPower, -1.0, 1.0);
        bucketElevationMotor.setPower(bucketElevationPower);
        bucketElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void SetBucketLiftPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        bucketElevationMotor.setTargetPosition(encoderTargetPosition);
        bucketElevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }

    public void ElevateCollector(double elevationPower) {
        elevationPower = com.qualcomm.robotcore.util.Range.clip(elevationPower, -1.0, 1.0);
        if (Math.abs(elevationPower) < 0.05) {
            elevationPower = 0;
        }
        mineralCollectionElevationMotor.setPower(elevationPower);
        mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetElevatorCollectioPositionByEncoder(int ElevatorCollectioPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        mineralCollectionElevationMotor.setTargetPosition(ElevatorCollectioPosition);
        mineralCollectionElevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }

    public void RotateCollector(double collectionPower) {
        collectionPower = com.qualcomm.robotcore.util.Range.clip(collectionPower, -1.0, 1.0);
        if (Math.abs(collectionPower) < 0.1) {
            collectionPower = 0;
        }
        mineralCollectionMotor.setPower(collectionPower);
    }

}
