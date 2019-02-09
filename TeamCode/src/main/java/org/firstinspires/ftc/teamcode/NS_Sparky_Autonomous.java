package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//@Autonomous (name = "Sparky Autonomous Mode", group = "Competition")
public abstract class NS_Sparky_Autonomous extends NS_Robot_Sparky {
    private double headingAngle = -40;
    private double sweepAngle = 50; // degrees
    private double deltaAngle = 5; // turn delta for camera sweep
    private double headingSpeed = NS_Sparky_Manual.PowerRegulator.ONEFOURTH;
    private double liftSpeed = NS_Sparky_Manual.PowerRegulator.FULL;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AXFmWqD/////AAAAGcQnWrSZvE1AhYt9fSvjbLhCIaNZZUuskVeDd4leXiCP1m03vRltyNPflJVYp8aCNJWor3BBw1OSVr+ykNY6LBTrakg4pDgBBl+GP08GRwlaGdYHGRwMayINjNZfXEflnGzt4tERZA7Dab+c5pNsyn3EvyMmFabBg7LHefKWWkdP489G2/g0Pj7BDITZfiUCFlTMl0Zzv3SLQIA2ri8u77/FbfgKF5UrqLH0Am7rN3Q8l6BNBktQ69itm867v06ENiwzHRBNkGymjT2arEWwp43rL3JjxKMg8XP+75YIyfEAJ/87lBCHfAODClGmTITQu42UgFBAQUmnIVb/SVMZnKALPrdMPuUG+LMZwOgGe4f1";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private VuforiaTrackable lastTrackable = null;
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /* Variable to store our instance of the Vuforia localization engine.
     */
    private VuforiaLocalizer vuforia;
    VuforiaTrackables targetsRoverRuckus;
    List<VuforiaTrackable> allTrackables;


    /* TersorFlow detection variables
     */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    // Variable to store instance of the Tensor Flow Object Detection engine
    private TFObjectDetector tfod;
    private enum MineralPosition {
        POS_NONE,
        POS_LEFT,
        POS_CENTER,
        POS_RIGHT
    }
    MineralPosition goldPosition = MineralPosition.POS_NONE;

    // Abstract method to be implemented by autonomous modes
    public abstract void PreProgrammedPlay() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        SparkyInitialize();
        SparkyVuforiaInitialize();
        SparkyTFODInitialize();
        SparkyTFODActivate();
        SparkyStart();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


        /*
         * Turn so that camera faces the rightmost mineral
         * Scan from right to left to locate the gold mineral
         * If gold mineral is found, then
         *      knock off gold mineral
         *      go the same distance backwards
         * End If
         * Turn so that camera is facing towards the Vuforia target image
         * Drive forward so that robot stops within a foot from Vuforia target
         * If Vuforia target gets detected, then
         *      Turn so that back of robot is facing the depot
         *      Back up to depot
         *      Dump team marker in depot
         * Else
         *      PreProgrammedPlay
         * End If
         * Extend color sensor arm
         * Drive forward until color sensor senses the black crater rim
         * Stop Robot
         */
        AutonomousStart();

        //SetCargoBucketPositionByEncoder(bucketElevationRestPosition, NS_Sparky_Manual.PowerRegulator.ONEFOURTH );

        SparkyTFODSampleMineral();
        SparkyTFODShutdown();


        AutonomousPlay();

        //SparkyVuforiaActivate();
        //SparkyVuforiaAcquire();

        //SparkyVuforiaActivate();

        // Acquire position of target images
        //SparkyVuforiaAcquire();
        /*
        If target detected then
            calculate the distance based on Vuforia
            move based on calculated values
        else
            move based on pre-programmed instructions
            which will need separate opmodes
         */
        /*
        if (targetVisible == true) {
            // Move based on calculations from Vuforia inputs
        }
        else {
            // Move based on pre-programmed instructions
        }
        */

        SparkyStop();
    }


    /******
     * AUTONOMOUS
     * @throws InterruptedException
     */
    public void AutonomousStart() throws InterruptedException {
        SetCargoLiftPositionByEncoder(cargoLiftEncoderPulsesHighPosition, NS_Sparky_Manual.PowerRegulator.FULL);
        WaitWhileBusy();
        GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEFIFTH, -20);
        WaitWhileBusy();
        SetCargoLiftPositionByEncoder(cargoLiftEncoderPulsesLowPosition, NS_Sparky_Manual.PowerRegulator.FULL);
        // Following wait commented to reduce time
        //WaitWhileBusy();
    }

    public void AutonomousPlay() throws InterruptedException {
        PreProgrammedPlay();
    }

    public void AutonomousDepotClaim() throws InterruptedException {
        ActuateAutonomousServo(teamMarkerServoPositionDump);
        sleep(800);
        ActuateAutonomousServo(teamMarkerServoPositionRest);
        sleep(800);
    }


    /******
     * VUFORIA
     */
    private void SparkyVuforiaInitialize() {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = (int) (1.5 * mmPerInch);   // 6.5 eg: Camera is mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = (int) (9.75 * mmPerInch);   // 8.5 eg: Camera is  mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = (int) (1.75 * mmPerInch);     // 8.5 eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, -90));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }

    private void SparkyVuforiaActivate() {
        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    private void SparkyVuforiaAcquire() { //throws InterruptedException {
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        long timeoutPeriod = 5000; // milliseconds

        GyroTurn(0.1, 50);

        targetVisible = false;
        while (opModeIsActive()
                && targetVisible == false
                && ((currentTime - startTime) < timeoutPeriod)
                && headingAngle <= sweepAngle) {
            telemetry.addData("TFOD Aquire:","ST: %d,  CT: %d,  TO: %d", startTime, currentTime, timeoutPeriod);
            // check all the trackable target to see which one (if any) is visible.
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    lastTrackable = trackable;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                // Break out of the while loop
            } else {
                telemetry.addData("Visible Target", "none");
                // Turn the camera (in our case robot) to seek for the target
                headingAngle += deltaAngle;
                GyroTurn(headingSpeed, headingAngle);
                WaitWhileBusy();
            }
            telemetry.update();

            currentTime = System.currentTimeMillis();
        }
    }


    /******
     * TENSOR FLOW
     */
    private void SparkyTFODInitialize() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // Set our preferences
        tfodParameters.useObjectTracker = false;
        tfodParameters.minimumConfidence = 0.5;

        // Instantiate Tensor Flow object detection
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void SparkyTFODActivate() {
        if (tfod != null) tfod.activate();
    }

    private void SparkyTFODShutdown() {
        if (tfod != null) tfod.shutdown();
    }

    private void SparkyTFODSampleMineral() {
        Recognition goldRecognition = null;
        boolean goldAlignment = false;
        double alignmentThreshold = 25;
        double turnSpeed = 0.1;
        double currentDeviation = 10000, previousDeviation = 10000;
        ElapsedTime elapsedTimer = new ElapsedTime();
        double timeoutPeriod = 5000;
        elapsedTimer.reset();

        // Starting point for the algorithm
        GyroTurn(NS_Sparky_Manual.PowerRegulator.ONEHALF, -60);

        // Start scanning by turning the robot from right to left
        while (opModeIsActive() && (goldAlignment == false)
                && (elapsedTimer.time() < timeoutPeriod) && (GetCurrentAngle() < 50)) {

            RCDrive(-NS_Sparky_Manual.PowerRegulator.ONETENTH, NS_Sparky_Manual.PowerRegulator.ONETENTH);

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    //goldRecognition = null;
                    for (Recognition recogObj : updatedRecognitions) {
                        telemetry.addData("Current Angle", "%f", GetCurrentAngle());
                        if (recogObj.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            //goldRecognition = recogObj;
                            previousDeviation = currentDeviation;
                            currentDeviation = (recogObj.getImageWidth()/2)
                                    - ((recogObj.getLeft()+recogObj.getRight())/2);
                            telemetry.addData("GOLD FOUND:","PD:%f, CD:%f",
                                    previousDeviation, currentDeviation);
                            if (Math.abs(currentDeviation) > Math.abs(previousDeviation)) {
                                goldAlignment = true;
                                RCDrive(0.0, 0.0);
                            }
                        }
                        telemetry.addData("Minerals:","Type? %s",
                                (recogObj.getLabel().equals(LABEL_GOLD_MINERAL)) ? "Gold" : "Silver");
                        telemetry.addData("Pos", "L:%f, R:%f, B:%f, T:%f",
                                recogObj.getLeft(), recogObj.getRight(),
                                recogObj.getBottom(), recogObj.getTop());
                        telemetry.addData("Dim", "W:%f, H:%f, IW:%d, IH:%d",
                                recogObj.getWidth(), recogObj.getHeight(),
                                recogObj.getImageWidth(), recogObj.getImageHeight());
                    }
                    telemetry.update();
                }
            }
        }
        // After while loop, stop turning
        RCDrive(0, 0);

        if (goldAlignment == true) {
            double driveDistance = 0;
            if (Math.abs(GetCurrentAngle()) > 20) { driveDistance = 30; }
            else { driveDistance = 24; }
            GyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, driveDistance, 0.0);
            //WaitWhileDriving();
            GyroDrive(NS_Sparky_Manual.PowerRegulator.FULL, -driveDistance, 0.0);
           // WaitWhileDriving();
        }
        telemetry.addData("TFOD Sampling:", "End of Sampling");
        telemetry.update();
    }
}
