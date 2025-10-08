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

        double currentVelocity = 0.0;

        // THE DECLARING of the... MOTOR!!!!
        DcMotorEx launcherOne = hardwareMap.get(DcMotorEx.class,"launcherOne");

        // Reversing the motors.
        launcherOne.setDirection(DcMotorSimple.Direction.REVERSE);




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                launcherOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                launcherOne.setTargetPosition(2000);
                launcherOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                launcherOne.setPower(0.5);
            }

            if (gamepad1.b) {
                launcherOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                launcherOne.setTargetPosition(-2000);
                launcherOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                launcherOne.setPower(0.5);

            }

            if (gamepad1.y) {
                launcherOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.a) {
                launcherOne.setVelocity(15000);
            }

            if (gamepad2.b) {
                launcherOne.setVelocity(-15000);
            }

            if (gamepad2.y) {
                launcherOne.setVelocity(0);
            }

            currentVelocity = launcherOne.getVelocity();

            telemetry.addData("Current Velocity", currentVelocity);



            telemetry.update();
            
        }
    }
}
