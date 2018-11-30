package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class NS_Robot_Sparky extends LinearOpMode {
    // Encoder Calculations
    private final double encoderTotalPulses = 1440;
    private final double ratioMotorToPinion = 1;
    private final double pinionCircumference = 19.085 * Math.PI;
    private final double encoderPulsesPerMilimeter = encoderTotalPulses * ratioMotorToPinion / pinionCircumference;
    private final double cargoLiftBottomPosition = 0.0; //mm
    private final double cargoLiftTopPosition = 280; //279.4; //mm, 11 inches
    private final double cargoLift100 = 100;
    private final double cargoLift200 = 200;
    private final double cargoLiftForestallLimit = 10; //mm
    private final double cargoLiftLowPosition = cargoLiftBottomPosition + cargoLiftForestallLimit;
    private final double cargoLiftHighPosition = cargoLiftTopPosition - cargoLiftForestallLimit;
    final int encoderPulsesLowPosition = (int) (encoderPulsesPerMilimeter * cargoLiftLowPosition);
    final int encoderPulsesHighPosition = (int) (encoderPulsesPerMilimeter * cargoLiftHighPosition);
    final int encoderPulses100 = (int) (encoderPulsesPerMilimeter * cargoLift100);
    final int encoderPulses200 = (int) (encoderPulsesPerMilimeter * cargoLift200);

    //DriveTrain Motors
    private DcMotor treadLeftMotor = null;
    private DcMotor treadRightMotor = null;

    //Cargo Lift Motor
    private DcMotor cargoLiftMotor = null;
    //Mineral Transporting Motor and Servos
    private DcMotor bucketElevationMotor = null;
    private Servo bucketRotationServo = null;
    private Servo bucketSwivelServo = null;
    private Servo bucketWallServo = null;
    private Servo bucketPivotServo = null;

    //Lander Latch Motor and Servo
    private DcMotor latchRotationMotor = null;

    private Servo latchHookServo = null;

    //Mineral Collection Motor
    private DcMotor mineralCollectionMotor = null;

    public void CreateHardwareMap() {
        //treadLeftMotor = hardwareMap.dcMotor.get("treadLeftMotor");
        //treadRightMotor = hardwareMap.dcMotor.get("treadRightMotor");

        cargoLiftMotor = hardwareMap.dcMotor.get("cargoLiftMotor");

        /*bucketElevationMotor = hardwareMap.dcMotor.get("mineralElevationMotor");
        bucketRotationServo = hardwareMap.servo.get("mineralRotationServo");
        bucketSwivelServo = hardwareMap.servo.get("mineralSwivelServo");
        bucketPivotServo = hardwareMap.servo.get("bucketPivotServo");
        bucketWallServo = hardwareMap.servo.get("bucketWallServo");

        latchRotationMotor = hardwareMap.dcMotor.get("latchRotationMotor");
        latchHookServo = hardwareMap.servo.get("latchHookServo");

        mineralCollectionMotor = hardwareMap.dcMotor.get("mineralCollectionMotor");*/
    }

    //Initialization Method
    public void Initialization() {
        CreateHardwareMap();
        // Set the cargo lift to starting position
        cargoLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetCargoLiftPositionByEncoder(encoderPulsesLowPosition,0.25);
    }

    //Driving Method
    /*public void TankDrive(double leftPower, double rightPower) {
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        treadLeftMotor.setPower(leftPower);
        treadRightMotor.setPower(rightPower);
    }

    public void RotateCollector(double collectionPower) {
        double max = Math.abs(collectionPower);
        if (max > 1.0) {
            collectionPower /= max;
        }
        mineralCollectionMotor.setPower(collectionPower);
    }*/

    public void SetCargoLiftPositionByEncoder(int encoderTargetPosition, double liftSpeed) {
        // Set Target and Turn On RUN_TO_POSITION
        cargoLiftMotor.setTargetPosition(encoderTargetPosition);
        cargoLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //cargoLiftpower = Range.clip(Math.abs(liftPower), 0.0, 1.0);
        cargoLiftMotor.setPower(liftSpeed);
    }



}
