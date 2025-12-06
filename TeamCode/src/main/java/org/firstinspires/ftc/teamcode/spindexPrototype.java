package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "spindexPrototype")
//@Disabled
public class spindexPrototype extends LinearOpMode {
    TouchSensor limitOne;
    TouchSensor limitTwo;

    private CRServo spindexServo;

    //dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        boolean limitCondition1 = true;

        boolean launching = false;

        boolean forceRun = true;

        boolean toLaunch = false;

        boolean running = false;

        boolean runningTwo = false;

        boolean wantToLaunch = false;

        int sleepCounter = 0;
        boolean isSleeping = false;


        spindexServo = hardwareMap.get(CRServo.class, "spindexServo");

        Servo spatulaServo = hardwareMap.servo.get("spatulaServo");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");

        //When looking up documentation or CR Servos, I found this line. Hoping this might resets the servo to it starting position. Needs testing.
        //rotateServo.resetDeviceConfigurationForOpMode();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {




            if (gamepad1.dpadRightWasPressed()) {
                forceRun = true;
            }

            if (launching || limitCondition1) spindexServo.setPower(0.1);
            //else if () spindexServo.setPower(0.75);
            else spindexServo.setPower(0);

            // if the limit switch is not pressed or an input is forcing it to run, then spin the spindexer
            if (!wantToLaunch && (!limitOne.isPressed() || forceRun)) {
                limitCondition1 = true;
                // potential issue in it wanting to go but launching preventing that, but wait to go off limit switch should fix it
                running = true;
            }
            else limitCondition1 = false;

            // waiting until outside the limit switch, then letting it stop when it hits a switch again
            if (running && !limitOne.isPressed()) {
                running = false;
                forceRun = false;
            }

            // same thing but for the launching position
            if (wantToLaunch && !limitTwo.isPressed()) {
                launching = true;
                // runningTwo = true;
            }
            else launching = false;

//            if (runningTwo && !limitTwo.isPressed()) {
//                runningTwo = false;
//                toLaunch = false;
//            }

            if (gamepad1.a) {
                wantToLaunch = true;
            } else if (gamepad1.b) {
                wantToLaunch = false;
            }


            if (gamepad2.left_bumper) {
                spatulaServo.setPosition(-1);
            } else if (gamepad2.right_bumper && !limitCondition1 && !launching) {
                spatulaServo.setPosition(1);
            }

            if (gamepad2.back && !limitCondition1 && !launching) {
                spatulaServo.setPosition(1);
                isSleeping = true;
                sleepCounter = 0;
            }
            if (isSleeping && sleepCounter >= 30) {
                isSleeping = false;
                spatulaServo.setPosition(-1);

            }
            sleepCounter ++;

            telemetry.addData("Servo power: ", spindexServo.getPower());
            telemetry.addData("Magnetic Switch 1 is letting servo run: ", limitCondition1);
            telemetry.addData("Not in launching position: ", launching);
            // telemetry.addData(": ", toLaunch);
            telemetry.addData("Going to next limit: ", forceRun);

            // next line of telemetry needed in drive code:
            telemetry.addData("In launching mode: ", wantToLaunch);
            telemetry.update();
        }
    }
}
