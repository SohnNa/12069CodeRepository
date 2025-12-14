package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "v4BotTeleOp")
@Disabled
public class v4BotTeleOp extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    TouchSensor limitOne;

    TouchSensor limitTwo;

    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
    String artifactStatus = "";
    int counter = 0;
    String artifactOne;

    String artifactTwo;

    String artifactThree;

    double speed;

    private ElapsedTime runtime = new ElapsedTime();

    double prevTime = 0.0;

    int next_pos = 0;

    boolean launching = false;

    boolean spatulaInWay = false;

    boolean toFirstPos = true;

    int SpindexPos = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        DcMotorEx spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");

        Servo spatulaServo = hardwareMap.servo.get("spatulaServo");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");



        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");


        // Reversing the motors.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTagWebcam.init(hardwareMap, telemetry);

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
            } else if (rx < -1) {
                rx = -1;
            }

            if (gamepad2.y) {
                intakeMotor.setVelocity(2800);
            } else if (gamepad2.b) {
                intakeMotor.setVelocity(-2800);
            } else {
                intakeMotor.setVelocity(0);
            }

            if (gamepad1.x) {
                flywheel.setVelocity(2000);
            } else if (gamepad1.b) {
                flywheel.setVelocity(-1600);
            } else if (gamepad1.y) {
                flywheel.setVelocity(1600);
            } else {
                flywheel.setVelocity(0);
            }

            next_pos += (int) ((runtime.milliseconds() - prevTime) * turretMotor.getVelocity());
            if (gamepad1.left_bumper) {
                //if (next_pos < limit) limits
                turretMotor.setTargetPosition(next_pos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setVelocity(600);
            } else if (gamepad1.right_bumper) {
                turretMotor.setTargetPosition(next_pos);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setVelocity(-600);
            }


            if (gamepad1.leftStickButtonWasPressed()) {
                toFirstPos = true;
            }


            //Driver Controls. Speed control
            if (gamepad2.left_bumper) {
                speed = 0.5f;
            } else {
                speed = 1.0f;
            }

            if (gamepad2.right_bumper) {
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


            if (gamepad1.dpadRightWasPressed() && !toFirstPos && !spatulaInWay) {
                // rotating between intake positions
                SpindexPos += 96;
                spindexMotor.setTargetPosition(SpindexPos);
                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexMotor.setVelocity(100);
            } else if (gamepad1.dpadLeftWasPressed() && !toFirstPos && !spatulaInWay) {
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


            if (gamepad1.dpad_down) {
                spatulaServo.setPosition(-1);
                spatulaInWay = false;
            } else if (gamepad1.dpad_up && launching && spindexMotor.getVelocity() < 300) {
                spatulaServo.setPosition(1);
                spatulaInWay = true;
            }


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



            /*
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

             */

            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagsBySpecificId(20);
            aprilTagWebcam.displayDetectionTelemetry(id20);



            //Spindexer Telemetry
            //telemetry.addData("Spindexer speed", spindexMotor.getVelocity());
            telemetry.addData("Magnetic Switch is pressed", limitOne.isPressed());
            telemetry.addData("SpatulaInWay", spatulaInWay);
            //telemetry.addData("Spindexer Position", spindexMotor.getCurrentPosition());
            //telemetry.addData("Spindexer Target Position", spindexMotor.getTargetPosition());
            telemetry.addData("In launching position", launching);
            //Intake & Output motor Telemetry
            //telemetry.addData("Turret Velocity", turretMotor.getVelocity());
            //telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
            telemetry.addData("Flywheel", flywheel.getVelocity());
            //Color Sensor Telemetry
            telemetry.addData("Artifact Status", artifactStatus);
            /*
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            */
            // next line of telemetry needed in drive code:

            telemetry.update();


            frontLeft.setVelocity((frontLeftPower * 2700) * speed);
            backLeft.setVelocity((backLeftPower * 2700) * speed);
            frontRight.setVelocity((frontRightPower * 2700) * speed);
            backRight.setVelocity((backRightPower * 2700) * speed);

            prevTime = runtime.milliseconds();


        }
    }


}


