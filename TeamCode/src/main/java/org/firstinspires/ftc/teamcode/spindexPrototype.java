package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "spindexPrototype")
//@Disabled
public class spindexPrototype extends LinearOpMode {
    TouchSensor limitOne;
    TouchSensor limitTwo;

    private CRServo spindexServo;

    //dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        boolean limitCondition1 = false;

        boolean limitCondition2 = false;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        spindexServo = hardwareMap.get(CRServo.class, "spindexServo");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");

        //When looking up documentation or CR Servos, I found this line. Hoping this might resets the servo to it starting position. Needs testing.
        //rotateServo.resetDeviceConfigurationForOpMode();

        waitForStart();
        imu.resetYaw();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                spindexServo.setPower(0.35);
            } else if (gamepad1.dpad_left) {
                spindexServo.setPower(0.65);
            } else {
                spindexServo.setPower(0.5);
            }




            if (gamepad1.dpad_left && !limitCondition1) {
                spindexServo.setPower(0.35);
            } else if (gamepad1.dpad_right && !limitCondition1) {
                spindexServo.setPower(0.65);
            } else {
                spindexServo.setPower(0.5);
            }



            /*
            if (limitOne.isPressed) {
                limitCondition1 = true;
                telemetry.addData("Switch One", "On");
            } else {
                limitCondition1 = false;
                telemetry.addData("Switch One", "Off");
            }

            if (limitTwo.isPressed) {
                limitCondition2 = true;
                telemetry.addData("Switch Two", "On");
            } else {
                limitCondition2 = false;
                telemetry.addData("Switch Two", "Off");
            }
            */
            telemetry.addData("Servo power", spindexServo.getPower());


            telemetry.update();
        }
    }
}

