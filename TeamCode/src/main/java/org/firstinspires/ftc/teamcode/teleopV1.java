


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



@TeleOp(name = "teleopV1")
//@Disabled
public class teleopV1 extends LinearOpMode {
    

    //dashboard = FtcDashboard.getInstance();
    DistanceSensor distance_1;
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
        

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        

        waitForStart();
        imu.resetYaw();
        //This finds the desired way the robot should point at the start...yeah
        double desiredBotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
           
           
           
            //if (rx != 0) {
            //    desiredBotHeading = botHeading; 
            //}
            //else {
            //    rx = (botHeading - desiredBotHeading)/(Math.PI/2);
            
            if (rx > 1) {
                rx = 1;
            }
            else if (rx < -1) {
                rx = -1; 
            }

            //}
            
            
            
            
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
            
            
            
        /*            
            //This does work...Makes the robot be stuck between 100% and 50%. Locks out the 
            //other buttons. 
            
            if (gamepad1.right_bumper || test) {
                speed = 0.5f;
                test2 = "On";
            } else {
                speed = 1.0f;
                test2 = "Off";
                test = false;
            }
           
          */
            
            
            telemetry.update();
                
            
            
            //Total power calculations. 
            
            frontLeft.setVelocity(frontLeftPower * 5000 * speed);
            backLeft.setVelocity(backLeftPower * 5000 * speed);
            frontRight.setVelocity(frontRightPower * 5000 * speed);
            backRight.setVelocity(backRightPower * 5000 * speed);
        }
    }
}
