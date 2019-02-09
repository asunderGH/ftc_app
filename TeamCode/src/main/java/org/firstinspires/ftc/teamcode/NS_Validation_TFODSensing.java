/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Tensor Flow Test", group = "Validation")
//@Disabled
public class NS_Validation_TFODSensing extends NS_Robot_Sparky {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        SparkyInitialize();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        SparkyStart();
        WaitWhileBusy();

        GyroTurn(0.1, -50);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            Recognition goldRecognition = null;
            boolean goldAlignment = false;
            double alignmentThreshold = 25;
            double turnSpeed = 0.06;
            double currentDeviation = 10000, previousDeviation = 10000;

            ElapsedTime elapsedTimer = new ElapsedTime();
            long timeoutPeriod = 5000;
            elapsedTimer.reset();
            // Start scanning by turning the robot from right to left
            while (opModeIsActive() && (goldAlignment == false)
                    && (elapsedTimer.time() < timeoutPeriod) && (GetCurrentAngle() < 50)) {
                if (gamepad1.a == true) { alignmentThreshold = 5.0; }
                else if (gamepad1.b == true) { alignmentThreshold = 10.0; }
                else if (gamepad1.x == true) { alignmentThreshold = 15.0; }
                else if (gamepad1.y == true) { alignmentThreshold = 20.0; }

                if (gamepad1.dpad_down == true){ turnSpeed = 0.05;}
                else if (gamepad1.dpad_left == true){ turnSpeed = 0.1;}
                else if (gamepad1.dpad_right == true){ turnSpeed = 0.15;}
                else if (gamepad1.dpad_up == true){ turnSpeed = 0.2;}

                RCDrive(-turnSpeed, turnSpeed);
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
            // After while loop, turn off tfod
            RCDrive(0, 0);

            if (goldAlignment == true) {
                double driveDistance = 0;
                if (Math.abs(GetCurrentAngle()) > 20) { driveDistance = 30; }
                else { driveDistance = 24; }
                GyroDrive(0.2, driveDistance, 0.0);
                WaitWhileBusy();
                GyroDrive(0.2, -driveDistance, 0.0);
                WaitWhileBusy();
            }

            GyroTurn(0.1, 50);

        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // Set our preferences
        tfodParameters.useObjectTracker = false;
        tfodParameters.minimumConfidence = 0.4;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
