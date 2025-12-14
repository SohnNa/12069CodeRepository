/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AprilTagTeleOp", group = "Concept")
//@Disabled
public class aFTCDriveToAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */

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


    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initAprilTag();


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

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                telemetryAprilTag();

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

                // Push telemetry to the Driver Station.


                // Save CPU resources; can resume streaming when needed.
                //if (gamepad1.dpad_down) {
                //    visionPortal.stopStreaming();
               // } else if (gamepad1.dpad_up) {
                //    visionPortal.resumeStreaming();
               // }
//
                // Share the CPU.
               // sleep(20);

            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "webcamera"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class
