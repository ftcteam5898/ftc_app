package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by William.Beard on 9/27/2017.
 *
 * Hello, future Lakeview robotics programmer.
 * This code is a basic outline for an
 * autonomous class. I am trying to design
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

@Disabled
@Autonomous(name="MainAutonomous", group="Autonomous")
public class MainAutonomous extends LinearOpMode {

    private DcMotor motorLeftBack = null, motorRightBack = null, motorRightFront = null, motorLeftFront = null;
    private ColorSensor colorSensor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Motor declarations for holonomic drive.
        motorLeftBack = hardwareMap.get(DcMotor.class, "mlb");
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack = hardwareMap.get(DcMotor.class, "mrb");
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront = hardwareMap.get(DcMotor.class, "mlf");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront = hardwareMap.get(DcMotor.class, "mrf");
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        waitForStart();
        while (opModeIsActive()) {
            //This is where stuff goes down...
        }
    }

    public void runForTime(int setting, int time, double power) {
        //Time in milliseconds
        //Settings below...
        //1 = runForward
        //2 = runBackward
        //3 = runRight
        //4 = runLeft
        //5 = turnRight
        //6 = turnLeft
        switch (setting) {
            case 1:
                runForward(power);
            case 2:
                runBackward(power);
            case 3:
                runRight(power);
            case 4:
                runLeft(power);
            case 5:
                turnRight(power);
            case 6:
                turnLeft(power);
        }
        sleep(time);
    }

    public boolean isRed() {
        //This method is designed to accurately respond to whether the visible color is red rather than blue.
        int sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += colorSensor.red() - colorSensor.blue();
            sleep(100);
        }
        return sum >= 400;
    }

    public void runForward(double power) {
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorRightFront.setPower(power);
    }

    public void runBackward(double power) {
        motorLeftBack.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightBack.setPower(-power);
        motorRightFront.setPower(-power);
    }

    public void runRight(double power) {
        motorLeftBack.setPower(-power);
        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorRightFront.setPower(-power);
    }

    public void runLeft(double power) {
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(-power);
        motorRightBack.setPower(-power);
        motorRightFront.setPower(power);
    }

    public void turnRight(double power) {
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(power);
        motorRightBack.setPower(-power);
        motorRightFront.setPower(-power);
    }

    public void turnLeft(double power) {
        motorLeftBack.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightBack.setPower(power);
        motorRightFront.setPower(power);
    }

    public void stopMotors() {
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
    }

}
