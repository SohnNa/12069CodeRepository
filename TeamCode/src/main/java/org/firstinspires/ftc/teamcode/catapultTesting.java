package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;



@TeleOp(name = "catapultTesting")
//@Disabled
public class catapultTesting extends LinearOpMode {


    //dashboard = FtcDashboard.getInstance()
    @Override
    public void runOpMode() throws InterruptedException {


        double speed = 1.0f;
        boolean test = false;
        String test2 = "Offline";


        // THE DECLARING of the... MOTORS!!!!!
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class,"backRight");


        //Servo servo_intake = hardwareMap.servo.get("servo_intake");
        //distance_1 = hardwareMap.get(DistanceSensor.class, "distance_1");


        // Reversing the motors.
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setTargetPosition(1000);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setPower(0.5);
            }
        }
    }
}
