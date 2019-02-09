package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Crater Sensor Test", group = "Validation")
public class NS_Validation_ColorSensor extends LinearOpMode {
    private ColorSensor craterSensor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        craterSensor = hardwareMap.colorSensor.get("craterSensor");

        waitForStart();
        while (opModeIsActive()) {
            double Red = craterSensor.red();
            double Green = craterSensor.green();
            double Blue = craterSensor.blue();

            telemetry.addData("RGB: ","Red:%d, Green:%d, Blue:%d",
                    craterSensor.red(), craterSensor.green(), craterSensor.blue());
            telemetry.update();
        }
    }
}
