package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@TeleOp(name = "spindexPrototype")
//@Disabled
public class spindexPrototype extends LinearOpMode {
    TouchSensor limitOne;
    TouchSensor limitTwo;

    private CRServo spindexServo;

    //dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        boolean launching = false;

        boolean spatulaInWay = false;

        int sleepCounter = 0;
        boolean isSleeping = false;

        boolean toFirstPos = true;

        int SpindexPos = 0;



        DcMotorEx spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        Servo spatulaServo = hardwareMap.servo.get("spatulaServo");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");

        //When looking up documentation or CR Servos, I found this line. Hoping this might resets the servo to it starting position. Needs testing.
        //rotateServo.resetDeviceConfigurationForOpMode();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // going to first limit
            if (toFirstPos) {
                if (!limitOne.isPressed()) {
                    spindexMotor.setVelocity(25);
                } else {
                    spindexMotor.setVelocity(0);
                    toFirstPos = false;
                    spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }



            if (gamepad1.dpadRightWasPressed() && !toFirstPos  && !spatulaInWay) {
                // rotating between intake positions
                SpindexPos += 96;
                spindexMotor.setTargetPosition(SpindexPos);
                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexMotor.setVelocity(100);
            }
            else if (gamepad1.dpadLeftWasPressed() && !toFirstPos  && !spatulaInWay) {
                // rotating between intake positions
                SpindexPos -= 96;
                spindexMotor.setTargetPosition(SpindexPos);
                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexMotor.setVelocity(100);
            }





            if (gamepad1.aWasPressed() && !spatulaInWay) {
                if (launching) SpindexPos += 38;
                else SpindexPos += 58;
                spindexMotor.setTargetPosition(SpindexPos);
                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexMotor.setVelocity(50);
                launching = !launching;
            }


            if (gamepad2.left_bumper) {
                spatulaServo.setPosition(-1);
                spatulaInWay = false;
            } else if (gamepad2.right_bumper && launching && spindexMotor.getVelocity() < 300) {
                spatulaServo.setPosition(1);
                spatulaInWay = true;
            }

            if (gamepad2.back && launching && spindexMotor.getVelocity() < 300) {
                spatulaServo.setPosition(1);
                spatulaInWay = true;
                isSleeping = true;
                sleepCounter = 0;
            }
            if (isSleeping && sleepCounter >= 30) {
                isSleeping = false;
                spatulaServo.setPosition(-1);
                spatulaInWay = false;

            }
            sleepCounter ++;

            telemetry.addData("Spindexer speed", spindexMotor.getVelocity());
            telemetry.addData("Magnetic Switch is pressed", limitOne.isPressed());
            telemetry.addData("SpatulaInWay", spatulaInWay);
            telemetry.addData("Spindexer Position", spindexMotor.getCurrentPosition());
            telemetry.addData("Spindexer Target Position", spindexMotor.getTargetPosition());


            // next line of telemetry needed in drive code:
            telemetry.addData("In launching position", launching);
            telemetry.update();
        }
    }
}
/*
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

            if (runningTwo && !limitTwo.isPressed()) {
                runningTwo = false;
                toLaunch = false;
            }
            */