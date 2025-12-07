package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "spindexPrototypeNoLimits")
//@Disabled
public class spindexPrototypeNoLimits extends LinearOpMode {
    TouchSensor limitOne;
    TouchSensor limitTwo;

    private CRServo spindexServo;

    //dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        boolean limitCondition1 = false;

        boolean limitCondition2 = false;

        boolean launching = true;

        spindexServo = hardwareMap.get(CRServo.class, "spindexServo");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");

        //When looking up documentation or CR Servos, I found this line. Hoping this might resets the servo to it starting position. Needs testing.
        //rotateServo.resetDeviceConfigurationForOpMode();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                spindexServo.setPower(0.75);
            } else if (gamepad1.dpad_left) {
                spindexServo.setPower(-0.75);
            } else {
                spindexServo.setPower(0);
            }




            /*
            if (gamepad1.dpad_left && !limitCondition1 && !launching) {
                spindexServo.setPower(0.75);
            } else if (gamepad1.dpad_right && !limitCondition1 && !launching) {
                spindexServo.setPower(-0.75);
            } else if (gamepad1.dpad_right && !limitCondition2 && launching) {
                spindexServo.setPower(-0.75);
            } else {
                spindexServo.setPower(0);
            }

             */


            if (limitOne.isPressed()) {
                limitCondition1 = true;
            } else {
                limitCondition1 = false;
            }

            if (limitTwo.isPressed()) {
                limitCondition2 = true;
            } else {
                limitCondition2 = false;
            }

            if (gamepad1.a) {
                launching = true;
            } else if (gamepad1.b) {
                launching = false;
            }

            telemetry.addData("Servo power", spindexServo.getPower());
            telemetry.addData("Magnetic Switch 1 is on = ", limitCondition1);
            telemetry.addData("Magnetic Switch 2 is on = ", limitCondition2);
            telemetry.addData("Loading State = ", launching);
            telemetry.update();
        }
    }
}
