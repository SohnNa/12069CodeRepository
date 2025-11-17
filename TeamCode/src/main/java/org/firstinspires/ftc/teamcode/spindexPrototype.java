package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "spindexTesting")
//@Disabled
public class spindexPrototype extends LinearOpMode {
    TouchSensor limitOne; 
    TouchSensor limitTwo;

    private CRServo rotateServo;

    //dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        
            
        


        rotateServo = hardwareMap.get(CRServo.class, "rotateServo");

        limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
        limitTwo = hardwareMap.get(TouchSensor.class, "limitTwo");

        
        // Reversing the motors.

        

ion is assumed to be logo up / USB forward
        imu.initialize(parameters);
        

      
        waitForStart();
        imu.resetYaw();
        //This finds the desired way the robot should point at the start...yeah
       
        if (isStopRequested()) return;

        while (opModeIsActive()) {



        }
