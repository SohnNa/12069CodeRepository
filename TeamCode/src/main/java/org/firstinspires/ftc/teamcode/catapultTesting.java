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



@TeleOp(name = "All Purpose Motor Testing")
@Disabled
public class catapultTesting extends LinearOpMode {


    //dashboard = FtcDashboard.getInstance()
    @Override
    public void runOpMode() throws InterruptedException {

        double currentVelocity = 0.0;

        double currentVelocity2 = 0.0;

        // THE DECLARING of the... MOTOR!!!!
        DcMotorEx launcherOne = hardwareMap.get(DcMotorEx.class,"launcherOne");
        DcMotorEx launcherTwo = hardwareMap.get(DcMotorEx.class,"launcherTwo");

        // Reversing the motors.





        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                launcherOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                launcherOne.setTargetPosition(150);
                launcherOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                launcherOne.setPower(0.5);
            }

            if (gamepad1.b) {
                launcherOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                launcherOne.setTargetPosition(-150);
                launcherOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                launcherOne.setPower(0.5);

            }

            if (gamepad1.y) {
                launcherOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.a) {
                launcherOne.setVelocity(15000);
                launcherTwo.setVelocity(-15000);
            }

            if (gamepad2.b) {
                launcherOne.setVelocity(-15000);
                launcherTwo.setVelocity(15000);
            }

            if (gamepad2.y) {
                launcherOne.setVelocity(0);
                launcherTwo.setVelocity(0);
            }

            currentVelocity = launcherOne.getVelocity();

            currentVelocity2 = launcherTwo.getVelocity();

            telemetry.addData("Motor 1 Velocity", currentVelocity);

            telemetry.addData("Motor 2 Velocity", currentVelocity2);

            telemetry.update();
            
        }
    }
}
