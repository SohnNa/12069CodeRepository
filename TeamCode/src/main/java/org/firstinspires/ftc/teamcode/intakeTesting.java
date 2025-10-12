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



@TeleOp(name = "Intake Testing")
//@Disabled
public class intakeTesting extends LinearOpMode {


    //dashboard = FtcDashboard.getInstance()
    @Override
    public void runOpMode() throws InterruptedException {

        double currentVelocity = 0.0;

        double velocity = 0.0;

        double gearChanger = 1;

        // THE DECLARING of the... MOTOR!!!!
        DcMotorEx launcherOne = hardwareMap.get(DcMotorEx.class,"launcherOne");

        // Reversing the motors.
        launcherOne.setDirection(DcMotorSimple.Direction.REVERSE);




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                launcherOne.setVelocity(2680);
            }
            else if (gamepad1.b) {
                launcherOne.setVelocity(2680 * 0.8);
            }

            if (gamepad1.y) {
                launcherOne.setVelocity(2680 * 0.6);
            }

            if (gamepad1.x) {
                launcherOne.setVelocity(2680 * 0.4);
            }






            currentVelocity = launcherOne.getVelocity();

            telemetry.addData("Current Velocity", currentVelocity);



            telemetry.update();
            
        }
    }
}
