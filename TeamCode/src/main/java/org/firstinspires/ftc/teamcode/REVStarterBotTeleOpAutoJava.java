package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Autonomous Code(s)", group = "competition")
public class REVStarterBotTeleOpAutoJava extends LinearOpMode {

  private DcMotorEx flywheel;

  private DcMotorEx frontLeft;
  private Servo spatulaServo;
  private DcMotorEx frontRight;

  private DcMotorEx backRight;

  private DcMotorEx backLeft;

  private DcMotorEx spindexMotor;

  TouchSensor limitOne;
  TouchSensor limitTwo;

  private static final String TELEOP = "RedBackwardsW/launcher";
  private static final String AUTO_BLUE = "DriveForward";
  private static final String AUTO_RED = " BlueBackwardsW/launcher";
  private String operationSelected = TELEOP;

  boolean launching = false;

  boolean spatulaInWay = false;

  int sleepCounter = 0;
  boolean isSleeping = false;

  boolean toFirstPos = true;

  int SpindexPos = 0;

  @Override
  public void runOpMode() {
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

    frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
    backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
    backRight = hardwareMap.get(DcMotorEx.class,"backRight");

    spindexMotor = hardwareMap.get(DcMotorEx.class, "spindexMotor");
    limitOne = hardwareMap.get(TouchSensor.class, "limitOne");
    spatulaServo = hardwareMap.get(Servo.class, "spatulaServo");

  // Establishing the direction and mode for the motors
    frontLeft.setDirection(DcMotor.Direction.REVERSE);

    //Ensures the servo is not in the way of the spindexer.
    spatulaServo.setPosition(-1);

  //On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or DriveForward
    while (opModeInInit()) {
      operationSelected = selectOperation(operationSelected, gamepad1.guide);
      telemetry.update();

      if (toFirstPos) {
        if (!limitOne.isPressed()) {
          spindexMotor.setVelocity(25);
        } else {
          spindexMotor.setVelocity(0);
          toFirstPos = false;
          spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
      }
    }

    waitForStart();
    if (operationSelected.equals(AUTO_BLUE)) {
      doDriveForward();
    } else if (operationSelected.equals(AUTO_RED)) {
      doBlueAuto();
    } else {
      doRedAuto();
    }
  }

  /**
   * If the PS/Home button is pressed, the robot will cycle through the OpMode options following the if/else statement here.
   * The telemetry readout to the Driver Station App will update to reflect which is currently selected for when "play" is pressed.
   */
  private String selectOperation(String state, boolean cycleNext) {
    if (cycleNext) {
      if (state.equals(TELEOP)) {
        state = AUTO_BLUE;
      } else if (state.equals(AUTO_BLUE)) {
        state = AUTO_RED;
      } else if (state.equals(AUTO_RED)) {
        state = TELEOP;
      } else {
        telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
      }
    }
    telemetry.addLine("Press Home Button to cycle options");
    telemetry.addData("CURRENT SELECTION", state);
    if (state.equals(AUTO_BLUE) || state.equals(AUTO_RED)) {
      telemetry.addLine("Please remember to enable the AUTO timer!");
    }
    telemetry.addLine("Press START to start your program");
    return state;
  }
  
  //TeleOp Code 
  
  /**
   * If TeleOp was selected or defaulted to, the following will be active upon pressing "play".
   */
  private void doRedAuto() {
    int spindexPos = 0;
    // do we need this below?
    waitForStart();

    if (opModeIsActive()) {
      //RedAuto
      spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontLeft.setTargetPosition(100);
      backLeft.setTargetPosition(100);
      frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeft.setVelocity(400);
      backLeft.setVelocity(400);
      frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontLeft.setVelocity(0);
      backLeft.setVelocity(0);
      sleep(2000);

      //  check if speed is right
      flywheel.setVelocity(1700);

      spindexPos += 68;
      spindexMotor.setTargetPosition(spindexPos);
      spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      spindexMotor.setVelocity(250);

      while (!((spindexMotor.getVelocity() < 30) && (spindexMotor.getTargetPosition() >= spindexMotor.getCurrentPosition() - 5) && (spindexMotor.getTargetPosition() <= spindexMotor.getCurrentPosition() + 5))) {
        sleep(100);
      }
      // if ((spindexMotor.getVelocity() < 30) && (spindexMotor.getTargetPosition() >= spindexMotor.getCurrentPosition() - 5) && (spindexMotor.getTargetPosition() <= spindexMotor.getCurrentPosition() + 5)) {
      int i = 0;
      while (i < 3) {
        spatulaServo.setPosition(1);
        sleep(1000);
        spatulaServo.setPosition(-1);
        sleep(1000);
        spindexPos += 96;
        spindexMotor.setTargetPosition(spindexPos);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(250);
        i++;
        sleep(1000);

      }
      flywheel.setVelocity(0);






    }
  }




//Autonomous Code
//For autonomous, the robot will launch the pre-loaded 3 balls then back away from the goal, turn, and back up off the launch line.
  
  /**
   * For autonomous, the robot is using a timer and encoders on the drivetrain to move away from the target. 
   * This method contains the math to be used with the inputted distance for the encoders, resets the elapsed timer, and
   * provides a check for it to run so long as the motors are busy and the timer has not run out. 
   */


  /**
   * Blue Alliance Autonomous
   * The robot will fire the pre-loaded balls until the 10 second timer ends. 
   * Then it will back away from the goal and off the launch line.
   */
  private void doDriveForward
  () {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      //Drive Forward.
      frontLeft.setPower(0.4);
      frontRight.setPower(-0.4);
      backLeft.setPower(-0.4);
      backRight.setPower(0.4);
      sleep(2000);
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
    }
  }

  /**
   * Red Alliance Autonomous
   * The robot will fire the pre-loaded balls until the 10 second timer ends. 
   * Then it will back away from the goal and off the launch line.
   */
  private void doBlueAuto() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();

      //Blue Auto
      frontLeft.setPower(-0.4);
      frontRight.setPower(-0.4);
      backLeft.setPower(-0.4);
      backRight.setPower(-0.4);
      sleep(2000);
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
      sleep(1000);
      SpindexPos += 68;
      spindexMotor.setTargetPosition(SpindexPos);
      spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      spindexMotor.setVelocity(250);
      sleep(1000);
      flywheel.setVelocity(1600);
      sleep(1000);
      if ((gamepad1.dpadUpWasPressed() && (spindexMotor.getVelocity() < 30) && (spindexMotor.getTargetPosition() >= spindexMotor.getCurrentPosition() - 5) && (spindexMotor.getTargetPosition() <= spindexMotor.getCurrentPosition() + 5))) {
        spatulaServo.setPosition(1);
        sleep(1000);
        frontLeft.setPower(-0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(0.4);
        sleep(2000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
      }
    }
  }
}




