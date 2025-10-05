package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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



        // THE DECLARING of the... MOTOR!!!!
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");

        // Reversing the motors.
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            if (gamepad1.a) {
                backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setTargetPosition(2000);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setPower(0.5);
            }

            if (gamepad1.b) {
                backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setTargetPosition(-2000);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setPower(0.5);

            }

            if (gamepad2.a) {
                backLeft.setVelocity(10000);
            }

            if (gamepad2.b) {
                backLeft.setVelocity(-10000);
            }

            if (gamepad2.y) {
                backLeft.setVelocity(0);
            }

            telemetry.update();
            
        }
    }
}
