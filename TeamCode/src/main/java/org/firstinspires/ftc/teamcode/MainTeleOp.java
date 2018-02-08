package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by William.Beard on 9/27/2017.
 *
 * Hello, future Lakeview robotics programmer.
 * This code is a basic outline for an
 * TeleOp class. I am trying to design
 * it in a way that will allow reusability in
 * the future beyond when I may be programming
 * this robot. I am not the best at commenting
 * code, but I will attempt to give them when
 * necessary.
 *
 * Currently, this code will be designed for
 * the 2017-2018 FTC competition. It is for
 * a holonomic drive robot, however, it should
 * be easily adapted.
 */

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {

    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront, motorLift;
    private Servo left, right, left1, right1, jewel;
    private boolean manual = false;
    private ModernRoboticsI2cGyro gyro = null;

    private class Move extends TimerTask {
        MainTeleOp MTO = null;
        public Move(MainTeleOp mto) {
            MTO = mto;
        }
        public void run() {
            MTO.setLiftPosition(-200);
        }
    }

    @Override
    public void runOpMode() {

        jewel = hardwareMap.get(Servo.class, "jewels");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        motorRightBack = hardwareMap.get(DcMotor.class, "mrb");
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront = hardwareMap.get(DcMotor.class, "mlf");
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront = hardwareMap.get(DcMotor.class, "mrf");
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        //Motor declarations for holonomic drive.
        motorLeftBack = hardwareMap.get(DcMotor.class, "mlb");
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorLift = hardwareMap.get(DcMotor.class, "ml");
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left = hardwareMap.get(Servo.class, "lefts");
        right = hardwareMap.get(Servo.class, "rights");
        left1 = hardwareMap.get(Servo.class, "lefts1");
        right1 = hardwareMap.get(Servo.class, "rights1");

        //Wait for the play button to be hit
        waitForStart();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        jewel.setPosition(0);

        setLiftPosition(1000);

        Timer time = new Timer();

        left.setPosition(0.5);
        right.setPosition(0.5);
        left1.setPosition(0.5);
        right1.setPosition(0.5);

        //BEGIN
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motorLift.setTargetPosition(motorLift.getCurrentPosition() + 50);
                motorLift.setPower(1);
            }
            if (gamepad1.y) {
                motorLift.setPower(0);
                motorLift.setDirection(DcMotor.Direction.REVERSE);
                motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.b) {
                motorLift.setPower(0);
            }
            if (gamepad1.x) {
                left.setPosition(0.7);
                right.setPosition(0.3);
                left1.setPosition(0.7);
                right1.setPosition(0.3);
            }
            if (gamepad1.dpad_up) {
                setLiftPosition(-4700);
            } else if (gamepad1.dpad_down) {
                setLiftPosition(0);
            }
            if (gamepad1.left_bumper) {
                left.setPosition(0.5);
                right.setPosition(0.5);
                left1.setPosition(0.5);
                right1.setPosition(0.5);
            } else if (gamepad1.right_bumper) {
                left.setPosition(1);
                right.setPosition(0);
                left1.setPosition(1);
                right1.setPosition(0);
                time.schedule(new Move(this), 200);

            }
            telemetry.addData("Lift position", motorLift.getCurrentPosition());
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.update();
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = -gamepad1.left_trigger + gamepad1.right_trigger;
            //Formulas for holonomic power
            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            //Make sure each is in the proper range and if not, make it
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);
            //Set the power to each motor
            motorRightFront.setPower(FrontRight);
            motorLeftFront.setPower(FrontLeft);
            motorLeftBack.setPower(BackLeft);
            motorRightBack.setPower(BackRight);
            telemetry.update();
        }
    }

    public void setLiftPosition(int i) {
        // 0 = bottom, -4300 = top
        motorLift.setTargetPosition(i);
        motorLift.setPower(0.5);
    }

}
