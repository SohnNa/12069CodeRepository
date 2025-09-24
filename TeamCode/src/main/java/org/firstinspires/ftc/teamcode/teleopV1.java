


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





@TeleOp(name = "Teleop_V1")
//@Disabled
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    
    DistanceSensor distance_1;
    @Override
    public void runOpMode() throws InterruptedException {
        
        
        double speed = 1.0f; 
        boolean test = false; 
        String test2 = "Offline";       
        
        
        // THE DECLARING of the... MOTORS!!!!!
        DcMotorEx front_left = hardwareMap.get(DeMotorEx.class,_left");
        DcMotorEx back_left = hardwareMap.get(DeMotorEx.class,"back_left");
        DcMotorEx front_right = hardwareMap.get(DeMotorEx.class,"front_right");
        DcMotorEx back_right = hardwareMap.get(DeMotorEx.class,"back_right");
      
        //Servo servo_intake = hardwareMap.servo.get("servo_intake");
        //distance_1 = hardwareMap.get(DistanceSensor.class, "distance_1");

        
        // Reversing the motors. 
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
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
            double rx = gamepad1.right_stick_x
            
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
            double front_left_Power = (rotY + rotX + rx) / denominator;
            double back_left_Power = (rotY - rotX + rx) / denominator;
            double front_right_Power = (rotY - rotX - rx) / denominator;
            double back_right_Power = (rotY + rotX - rx) / denominator;
            
            
            
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
            
            front_left.setVelocity(front_left_Power * 10000 * speed);
            back_left.setVelocity(back_left_Power * 10000 * speed);
            front_right.setVelocity(front_right_Power * 10000 * speed);
            back_right.setVelocity(back_right_Power * 10000 * speed);
        }
    }
}
