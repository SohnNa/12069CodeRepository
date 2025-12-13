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



@TeleOp(name = "v3BotTeleOp")
//@Disabled
public class v3BotTeleOp extends LinearOpMode {
    double speed;
    TouchSensor limitOne;
    TouchSensor limitTwo;

    private ElapsedTime runtime = new ElapsedTime();

    double prevTime = 0.0;

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


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        runtime.reset();
        imu.resetYaw();
        //This finds the desired way the robot should point at the start...yeah
        double desiredBotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);






            /*
            if (rx != 0) {
                desiredBotHeading = botHeading;
            }
            else {
                rx = (botHeading - desiredBotHeading)/(Math.PI/2);
            }
            */


            if (rx > 1) {
                rx = 1;
            }
            else if (rx < -1) {
                rx = -1;
            }

            if (gamepad2.y) {
                intakeMotor.setVelocity(2800);
            } else if (gamepad2.b) {
                intakeMotor.setVelocity(-2800);
            } else {
                intakeMotor.setVelocity(0);
            }

            if (gamepad1.a) {
                flywheel.setVelocity(2800);
            } else if (gamepad1.x) {
                flywheel.setVelocity(-1600);
            } else if (gamepad1.b) {
                flywheel.setVelocity(1600);
            } else {
                flywheel.setVelocity(0);
            }
            next_pos = (int) ((runtime.milliseconds() - prevTime) * turretMotor.getVelocity());
            if (gamepad1.left_bumper) {
                //if () limits
                turretMotor.setTargetPosition(next_pos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setVelocity(600);
            } else if (gamepad1.right_bumper) {
                turretMotor.setTargetPosition(next_pos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setVelocity(-600);
            }




            //Driver Controls. Speed control
            if (gamepad2.right_bumper) {
                speed = 0.5f;
            } else {
                speed = 1.0f;
            }

            if (gamepad2.left_bumper) {
                imu.resetYaw();
            }








            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;



            telemetry.addData("Turret Velocity", turretMotor.getVelocity());
            telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
            //Color Sensor Telemetry
            //




            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());



            telemetry.update();






            //Total power calculations.




            frontLeft.setVelocity((frontLeftPower * 2700) * speed);
            backLeft.setVelocity((backLeftPower * 2700) * speed);
            frontRight.setVelocity((frontRightPower * 2700) * speed);
            backRight.setVelocity((backRightPower * 2700) * speed);

            prevTime = runtime.milliseconds();

        }
    }
}
