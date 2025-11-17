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


@TeleOp(name = "spindexTesting")
//@Disabled
public class spindexPrototype extends LinearOpMode {
    TouchSensor limitOne; 
    TouchSensor limitTwo;

    private CRServo rotateServo;

    //dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        boolean limitCondition1 = false;

        boolean limitCondition2 = false;
        
            
        


        rotateServo = hardwareMap.get(CRServo.class, "rotateServo");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");

        //When looking up documentation or CR Servos, I found this line. Hoping this might resets the servo to it starting position. Needs testing. 
        //rotateServo.resetDeviceConfigurationForOpMode();
      
        waitForStart();
        imu.resetYaw();
       
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                rotateServo.setPower(0.35);
            } else if (gamepad1.dpad_left) {
                rotateServo.setPower(0.65);
            } else {
                rotateServo.setPower(0.5);
            }



            /*
            if (gamepad1.dpad_left && !limitCondition1) {
                rotateServo.setPower(0.35);
            } else if (gamepad1.dpad_right && !limitCondition1) {
                rotateServo.setPower(0.65);
            } else {
                rotateServo.setPower(0.5);
            }


            */

            if (limitOne.isPressed) {
                limitCondition1 = true;
                telemetry.addData("Switch One", "On");
            } else {
                limitComdition1 = false;
                telemetry.addData("Switch One", "Off");
            }

            if (limitTwo.isPressed) {
                limitCondition2 = true;
                telemetry.addData("Switch Two", "On");
            } else {
                limitCondition2 = false;
                telemetry.addData("Switch Two", "Off");
            }

            telemetry.addData("Servo power", rotateServo.getPower());
            
            
            
            telemetry.update();
        }
