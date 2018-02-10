package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by William.Beard on 1/20/2018.
 */

@Autonomous(name="Blue", group="Autonomous")
public class Blue extends LinearOpMode {

    private Servo jewel;
    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront;
    private ColorSensor cs;

    @Override
    public void runOpMode() throws InterruptedException {
        cs = hardwareMap.get(ColorSensor.class, "cs");
        jewel = hardwareMap.get(Servo.class, "jewels");
        motorRightBack = hardwareMap.get(DcMotor.class, "mrb");
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront = hardwareMap.get(DcMotor.class, "mlf");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront = hardwareMap.get(DcMotor.class, "mrf");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack = hardwareMap.get(DcMotor.class, "mlb");
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        for (double d = 0.2; d <= 0.95; d += 0.05) {
            jewel.setPosition(d);
            if (!sleep(200)) return;
        }
        if (!sleep(1000)) return;
        int value = color();
        if (value >= 5) {
            motorLeftBack.setPower(0.6);
            motorLeftFront.setPower(0.6);
            motorRightBack.setPower(0.6);
            motorRightFront.setPower(0.6);
            if (!sleep(150)) return;
            jewel.setPosition(0);
            motorLeftBack.setPower(0);
            motorLeftFront.setPower(0);
            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
            if (!sleep(1000)) return;
            motorLeftBack.setPower(-0.6);
            motorLeftFront.setPower(-0.6);
            motorRightBack.setPower(-0.6);
            motorRightFront.setPower(-0.6);
            if (!sleep(150)) return;
        } else if (value <= -5) {
            motorLeftBack.setPower(-0.6);
            motorLeftFront.setPower(-0.6);
            motorRightBack.setPower(-0.6);
            motorRightFront.setPower(-0.6);
            if (!sleep(150)) return;
            jewel.setPosition(0);
            motorLeftBack.setPower(0);
            motorLeftFront.setPower(0);
            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
            if (!sleep(1000)) return;
            motorLeftBack.setPower(0.6);
            motorLeftFront.setPower(0.6);
            motorRightBack.setPower(0.6);
            motorRightFront.setPower(0.6);
            if (!sleep(150)) return;
        } else {
            jewel.setPosition(0);
        }

        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        if (!sleep(2000)) return;
        motorLeftBack.setPower(1);
        motorLeftFront.setPower(1);
        motorRightBack.setPower(-1);
        motorRightFront.setPower(-1);
        if (!sleep(1000)) return;
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);

    }

    public boolean sleep(int i) {
        if (!opModeIsActive()) return false;
        try {
            Thread.sleep(i);
        } catch (Exception e) {}
        return opModeIsActive();
    }

    public int color() {
        int rminusbsum = 0;
        for (int i = 0; i < 10; i++) {
            rminusbsum += (cs.red() - cs.blue());
            sleep(100);
        }
        return rminusbsum;
    }
}
