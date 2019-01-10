package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class NS_Robot_Sparky extends LinearOpMode {
    // Encoder Calculations
    private final double encoderTotalPulses = 1440;
    private final double pinionCircumference = 19.085 * Math.PI;

    private final double cargoLiftMotorToPinionRatio = 2;
    private final double encoderPulsesPerMilimeter = encoderTotalPulses * cargoLiftMotorToPinionRatio / pinionCircumference;
    private final double cargoLiftBottomPosition = 0.0; //mm
    private final double cargoLiftTopPosition = 170; //mm, 6.7 inches
    private final double cargoLiftForestallLimit = 2; //mm
    private final double cargoLiftLowPosition = cargoLiftBottomPosition + cargoLiftForestallLimit;
    private final double cargoLiftHighPosition = cargoLiftTopPosition - cargoLiftForestallLimit;
    public final int encoderPulsesLowPosition = (int) (encoderPulsesPerMilimeter * cargoLiftLowPosition);
    public final int encoderPulsesHighPosition = (int) (encoderPulsesPerMilimeter * cargoLiftHighPosition);

    private final double bucketElevationMotorToGearRatio = 2;
    private final double bucketElevationGearRotationsForCrater = 0.125; //45 Degrees from rest position
    private final double bucketElevationGearRotationsForDumping = 0.65;
    public final int bucketElevationRestPosition = 0;
    public final int bucketElevationCraterPosition =
            (int) (encoderTotalPulses * (bucketElevationMotorToGearRatio * bucketElevationGearRotationsForCrater));
    public final int bucketElevationDumpPosition =
            (int) (encoderTotalPulses * (bucketElevationGearRotationsForDumping * bucketElevationMotorToGearRatio));

    public final int mineralCollectionElevatorUpPosition = 0;
    public final int mineralCollectionElevatorDownPosition = (int) (encoderTotalPulses * 0.6);


    //DriveTrain Motors
    private DcMotor frontRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor backLeftMotor = null;

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
        //latchRotationMotor = hardwareMap.dcMotor.get("latchRotationMotor");
        //latchHookServo = hardwareMap.servo.get("latchHookServo");

        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        cargoLiftMotor = hardwareMap.dcMotor.get("cargoLiftMotor");

        bucketElevationMotor = hardwareMap.dcMotor.get("bucketElevationMotor");

        //mineralCollectionElevationMotor = hardwareMap.dcMotor.get("mineralCollectionElevationMotor");
        //mineralCollectionElevationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mineralCollectionMotor = hardwareMap.dcMotor.get("mineralCollectionMotor");
    }

    //Initialze_Sparky Method
    public void Initialze_Sparky() {
        CreateHardwareMap();
        //latchRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cargoLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

       // mineralCollectionElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void LatchArmPower(double latchMotorPower) {
        latchMotorPower = com.qualcomm.robotcore.util.Range.clip(latchMotorPower, -1.0, 1.0);
        if (Math.abs(latchMotorPower) < 0.1) {
            latchMotorPower = 0;
        }
        latchRotationMotor.setPower(latchMotorPower);
    }

    //Driving Method
    public void RCDrive(double leftSideMotorPower, double rightSideMotorPower) {
        double max = Math.max(Math.abs(leftSideMotorPower), Math.abs(rightSideMotorPower));
        if (max > 1.0) {
            leftSideMotorPower /= max;
            rightSideMotorPower /= max;
        }

        frontRightMotor.setPower(rightSideMotorPower);
        frontLeftMotor.setPower(leftSideMotorPower);
        backRightMotor.setPower(rightSideMotorPower);
        backLeftMotor.setPower(leftSideMotorPower);
    }

    public void ElevateCargoLift(double cargoLiftPower) {
        cargoLiftPower = Range.clip(cargoLiftPower, -1.0, 1.0);
        cargoLiftMotor.setPower(cargoLiftPower);
    }

    public void SetCargoLiftPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        cargoLiftMotor.setTargetPosition(encoderTargetPosition);
        cargoLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }


    public void ElevateCargoBucket(double cargoBucketLiftPower) {
        cargoBucketLiftPower = Range.clip(cargoBucketLiftPower, -1.0, 1.0);
        bucketElevationMotor.setPower(cargoBucketLiftPower);
    }

    public void SetCargoBucketPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        bucketElevationMotor.setTargetPosition(encoderTargetPosition);
        bucketElevationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }

    public void RotateMineralCollector(double collectionPower) {
        collectionPower = Range.clip(collectionPower, -1.0, 1.0);
        if (Math.abs(collectionPower) < 0.1) {
            collectionPower = 0;
        }
        mineralCollectionMotor.setPower(collectionPower);
    }

}
