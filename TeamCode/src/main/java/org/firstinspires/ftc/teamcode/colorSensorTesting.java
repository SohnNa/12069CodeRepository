package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;



@TeleOp(name = "colorSensorTesting")
//@Disabled
public class colorSensorTesting extends LinearOpMode {


    double speed;

    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
    String artifactStatus = "";

    TouchSensor limitOne;
    TouchSensor limitTwo;

    private ElapsedTime runtime = new ElapsedTime();

    double prevTime = 0.0;

    int counter = 0;

    String artifactOne;

    String artifactTwo;

    String artifactThree;

    int next_pos = 0;


    @Override
    public void runOpMode() throws InterruptedException {




        // THE DECLARING of the... MOTORS!!!!!
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class,"backRight");

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        DcMotorEx spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        Servo spatulaServo = hardwareMap.servo.get("spatulaServo");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");



        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");



        // Reversing the motors.
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();
        imu.resetYaw();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            green = colorSensor.green();
            red = colorSensor.red();
            blue = colorSensor.blue();

            if ((green < 100) && (red < 100) && (blue < 100)) {
                artifactStatus = "N/A or Hole";
            } else if (green > blue) {
                artifactStatus = "Green Artifact";
            } else if (blue > green) {
                artifactStatus = "Purple Artifact";
            }

            if (gamepad1.dpadRightWasPressed()) {
                if (counter == 0) {
                    artifactOne = artifactStatus;
                    counter += 1;
                } else if (counter == 1) {
                    artifactTwo = artifactStatus;
                    counter += 1;
                } else if (counter == 2) {
                    artifactThree = artifactStatus;
                    counter = 0;
                }
            }







            telemetry.addData("Artifact Status", artifactStatus);


            /*
            //Color Sensor Telemetry
            telemetry.addData("", "");
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());

             */



            telemetry.update();


        }
    }
}
