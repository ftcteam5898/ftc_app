package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by William.Beard on 1/30/2018.
 */

@Autonomous(name="NewBlue", group="Autonomous")
public class NewBlue extends LinearOpMode {

    private Servo jewel;
    private Servo left, right, left1, right1;
    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront;
    private DcMotor motorLift;
    private ColorSensor cs;
    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbAk0Qv/////AAAAGReJq1q3oEBBpL6gFlJ4tEtmB5O0nOAttAl5uqbuAxyfQdmBpSKniaAcsJo+qSLLEp/U9eOlrSyEj0vPBZz+fw9oatXAs9HH7Bvytz8M5NBokl/VZPfHhRKwVxc/E/SyxvwenQh/NrpWqHH1Jia2BLXEnqy0I4ANVepkbL4wZSYj+zhFrHx0UldPm+6XLJkKeZnf21674Wc00WeV9oVn9rpC8FTy6XOEgcgW01iYQPBJN6gLcyvTE5965w/rETGbzU8yPrdmcl1eCZBDH7vBDI1qBwOFCJSEKzBQ7Lc20gSZvvCTF/1mZN/jRXirc+t9KLGU+1k42yrfXL6fTWz3QZHt3eBSpwwN1mCJaFqi5soE";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        cs = hardwareMap.get(ColorSensor.class, "cs");
        jewel = hardwareMap.get(Servo.class, "jewels");
        motorRightBack = hardwareMap.get(DcMotor.class, "mrb");
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront = hardwareMap.get(DcMotor.class, "mlf");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront = hardwareMap.get(DcMotor.class, "mrf");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack = hardwareMap.get(DcMotor.class, "mlb");
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift = hardwareMap.get(DcMotor.class, "ml");
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left = hardwareMap.get(Servo.class, "lefts");
        right = hardwareMap.get(Servo.class, "rights");
        left1 = hardwareMap.get(Servo.class, "lefts1");
        right1 = hardwareMap.get(Servo.class, "rights1");
        waitForStart();
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        int max = 0;
        /*while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            max++;
        }*/
        telemetry.addData("Vuforia", vuMark.toString());
        telemetry.update();
        left.setPosition(0.5);
        right.setPosition(0.5);
        left1.setPosition(0.5);
        right1.setPosition(0.5);
        if (!opModeIsActive()) {return;}
        arm(800);
        if (!opModeIsActive()) {return;}
        for (double d = 0.2; d <= 1; d += 0.01) {
            jewel.setPosition(d); // Lower Jewel Arm
            if (!sleep(25)) return;
        }
        if (!opModeIsActive()) {return;}
        left.setPosition(1);
        right.setPosition(0);
        left1.setPosition(1);
        right1.setPosition(0);
        if (!opModeIsActive()) {return;}
        sleep(1000);
        if (!opModeIsActive()) {return;}
        arm(-1600);
        if (!opModeIsActive()) {return;}
        int i = color();
        if (!opModeIsActive()) {return;}
        if (i <= -1) {
            turn(-150);
            if (!opModeIsActive()) {return;}
            jewel.setPosition(0);
            if (!opModeIsActive()) {return;}
            turn(150);
            if (!opModeIsActive()) {return;}
        } else if (i >= 1) {
            turn(150);
            if (!opModeIsActive()) {return;}
            jewel.setPosition(0);
            if (!opModeIsActive()) {return;}
            turn(-150);
            if (!opModeIsActive()) {return;}
        } else {
            jewel.setPosition(0);
            if (!opModeIsActive()) {return;}
            sleep(500);
            if (!opModeIsActive()) {return;}
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            forward(2100);
            if (!opModeIsActive()) {return;}
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            forward(2600);
            if (!opModeIsActive()) {return;}
        } else {
            forward(3100);
            if (!opModeIsActive()) {return;}
        }
        turn(-1460);
        if (!opModeIsActive()) {return;}
        left.setPosition(0);
        right.setPosition(1);
        left1.setPosition(0);
        right1.setPosition(1);
        if (!opModeIsActive()) {return;}
        for (int a = 900; a <= 1100; a += 100) {
            forward(a);
            forward(-a);
        }
        if (!opModeIsActive()) {return;}
        forward(400);
        if (!opModeIsActive()) {return;}

    }

    public void arm(int i) {
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setTargetPosition(motorLift.getCurrentPosition() + i);
        motorLift.setPower(0.7);
        while (opModeIsActive() && motorLift.isBusy()) {}
        motorLift.setPower(0);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forward(int i) {
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + i);
        motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + i);
        motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() - i);
        motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() - i);
        motorLeftBack.setPower(0.7);
        motorLeftFront.setPower(0.7);
        motorRightBack.setPower(0.7);
        motorRightFront.setPower(0.7);
        while (opModeIsActive() && motorLeftFront.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy() && motorLeftBack.isBusy()) {}
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(int i) {
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + i);
        motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + i);
        motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + i);
        motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + i);
        motorLeftBack.setPower(0.3);
        motorLeftFront.setPower(0.3);
        motorRightBack.setPower(0.3);
        motorRightFront.setPower(0.3);
        while (opModeIsActive() && motorLeftFront.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy() && motorLeftBack.isBusy()) {}
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
