package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Autonomous Code(s)")
public class REVStarterBotTeleOpAutoJava extends LinearOpMode {

  private DcMotorEx flywheel;

  private DcMotor frontLeft;
  private CRServo servo;
  private DcMotor frontRight;

  private DcMotor backRight;

  private DcMotor backLeft;

  private static final int bankVelocity = 1300;
  private static final int farVelocity = 1900;
  private static final int maxVelocity = 2200;
  private static final String TELEOP = "RedBackwardsW/launcher";
  private static final String AUTO_BLUE = "DriveForward";
  private static final String AUTO_RED = " BlueBackwardsW/launcher";
  private String operationSelected = TELEOP;
  private double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);
  private ElapsedTime autoLaunchTimer = new ElapsedTime();
  private ElapsedTime autoDriveTimer = new ElapsedTime();

  @Override
  public void runOpMode() {
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    servo = hardwareMap.get(CRServo.class, "servo");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backLeft = hardwareMap.get(DcMotor.class,"backLeft");
    backRight = hardwareMap.get(DcMotor.class,"backRight");
    
  // Establishing the direction and mode for the motors
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    frontRight.setDirection(DcMotor.Direction.REVERSE);

    //Ensures the servo is active and ready
    servo.setPower(0);

  //On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or TeleOp
    while (opModeInInit()) {
      operationSelected = selectOperation(operationSelected, gamepad1.guide);
      telemetry.update();
      servo.setPower(1);
    }
    waitForStart();
    if (operationSelected.equals(AUTO_BLUE)) {
      doAutoBlue();
    } else if (operationSelected.equals(AUTO_RED)) {
      doAutoRed();
    } else {
      doTeleOp();
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
  private void doTeleOp() {
    if (opModeIsActive()) {
      /*
      while (opModeIsActive()) {
        // Calling our methods while the OpMode is running
        splitStickArcadeDrive();
        setFlywheelVelocity();
        manualCoreHexAndServoControl();
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Power", flywheel.getPower());
        telemetry.update();
      }

       */
      frontLeft.setPower(-0.4);
      frontRight.setPower(-0.4);
      backLeft.setPower(-0.4);
      backRight.setPower(-0.4);
      sleep(1000);
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
      sleep(1000);
      flywheel.setVelocity(1800);
      sleep(2000);
      servo.setPower(-1);
      sleep(3000);
      flywheel.setVelocity(0);
      frontLeft.setPower(-0.2);
      frontRight.setPower(0.2);
      backLeft.setPower(-0.2);
      backRight.setPower(0.2);
      sleep(1000);
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
      sleep(1000);
      frontLeft.setPower(-0.4);
      frontRight.setPower(-0.4);
      backLeft.setPower(-0.4);
      backRight.setPower(-0.4);
      sleep(1000);
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);





    }
  }
  
    /**
   * Controls for the drivetrain. The robot uses a split stick stlye arcade drive. 
   * Forward and back is on the left stick. Turning is on the right stick.
   */
  private void splitStickArcadeDrive() {
    float X;
    float Y;

    X = gamepad1.right_stick_x;
    Y = -gamepad1.left_stick_y;
    frontLeft.setPower(Y - X);
    frontRight.setPower(Y + X);
  }
  
    /**
   * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
   */
  private void manualCoreHexAndServoControl() {
    // Manual control for the Core Hex intake

    // Manual control for the hopper's servo
    if (gamepad1.dpad_left) {
      servo.setPower(1);
    } else if (gamepad1.dpad_right) {
      servo.setPower(-1);
    }
  }
  
    /**
   * This if/else statement contains the controls for the flywheel, both manual and auto.
   * Circle and Square will spin up ONLY the flywheel to the target velocity set.
   * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
   */
  private void setFlywheelVelocity() {
    if (gamepad1.back) {
      flywheel.setPower(-0.5);
    } else if (gamepad1.left_bumper) {
      FAR_POWER_AUTO();
    } else if (gamepad1.right_bumper) {
      BANK_SHOT_AUTO();
    } else if (gamepad1.b) {
      ((DcMotorEx) flywheel).setVelocity(bankVelocity);
    } else if (gamepad1.a) {
      ((DcMotorEx) flywheel).setVelocity(maxVelocity);
    } else {
      ((DcMotorEx) flywheel).setVelocity(0);

      // The check below is in place to prevent stuttering with the servo. It checks if the servo is under manual control!
      if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
        servo.setPower(0);
      }
    }
  }

//Automatic Flywheel controls used in Auto and TeleOp 

  /**
   * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
   * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
   * The servo will spin until the bumper is released.
   */
  private void BANK_SHOT_AUTO() {
    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
    servo.setPower(-1);
    if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 100) {

    } else {

    }
  }

  /**
   * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
   * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
   * The servo will spin until the bumper is released.
   */
  private void FAR_POWER_AUTO() {
    ((DcMotorEx) flywheel).setVelocity(farVelocity);
    servo.setPower(-1);
    if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {

    } else {

    }
  }

//Autonomous Code
//For autonomous, the robot will launch the pre-loaded 3 balls then back away from the goal, turn, and back up off the launch line.
  
  /**
   * For autonomous, the robot is using a timer and encoders on the drivetrain to move away from the target. 
   * This method contains the math to be used with the inputted distance for the encoders, resets the elapsed timer, and
   * provides a check for it to run so long as the motors are busy and the timer has not run out. 
   */
  private void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
    autoDriveTimer.reset();
    frontLeft.setTargetPosition((int) (frontLeft.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS));
    frontRight.setTargetPosition((int) (frontRight.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS));
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms) {
      idle();
    }
    frontLeft.setPower(0);
    frontRight.setPower(0);
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  /**
   * Blue Alliance Autonomous
   * The robot will fire the pre-loaded balls until the 10 second timer ends. 
   * Then it will back away from the goal and off the launch line.
   */


  private void doAutoBlue
  () {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
      // Fire balls
      //autoLaunchTimer.reset();
      //while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
        //BANK_SHOT_AUTO();
        //telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
        //telemetry.update();
      //}
      //((DcMotorEx) flywheel).setVelocity(0);

      //servo.setPower(0);
      // Back Up
      //autoDrive(0.5, -12, -12, 5000);
      // Turn
      //autoDrive(0.5, -8, 8, 5000);
      // Drive off Line
      frontLeft.setPower(0.4);
      frontRight.setPower(0.4);
      backLeft.setPower(0.4);
      backRight.setPower(0.4);
      sleep(1000);
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
  private void doAutoRed() {
    if (opModeIsActive()) {
      telemetry.addData("RUNNING OPMODE", operationSelected);
      telemetry.update();
        frontLeft.setPower(-0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(-0.4);
        sleep(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(1000);
        flywheel.setVelocity(1800);
        sleep(2000);
        servo.setPower(-1);
        sleep(3000);
        flywheel.setVelocity(0);
        frontLeft.setPower(0.2);
        frontRight.setPower(-0.2);
        backLeft.setPower(0.2);
        backRight.setPower(-0.2);
        sleep(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(1000);
        frontLeft.setPower(-0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(-0.4);
        sleep(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
  }
}


