package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Disabled
@Autonomous(name="BP1", group="Autonomous")
public class BP1 extends LinearOpMode {

    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront, motorLift;
    private Servo left, right, jewel, left1, right1;
    private ColorSensor cs;
    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbAk0Qv/////AAAAGReJq1q3oEBBpL6gFlJ4tEtmB5O0nOAttAl5uqbuAxyfQdmBpSKniaAcsJo+qSLLEp/U9eOlrSyEj0vPBZz+fw9oatXAs9HH7Bvytz8M5NBokl/VZPfHhRKwVxc/E/SyxvwenQh/NrpWqHH1Jia2BLXEnqy0I4ANVepkbL4wZSYj+zhFrHx0UldPm+6XLJkKeZnf21674Wc00WeV9oVn9rpC8FTy6XOEgcgW01iYQPBJN6gLcyvTE5965w/rETGbzU8yPrdmcl1eCZBDH7vBDI1qBwOFCJSEKzBQ7Lc20gSZvvCTF/1mZN/jRXirc+t9KLGU+1k42yrfXL6fTWz3QZHt3eBSpwwN1mCJaFqi5soE";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        cs = hardwareMap.get(ColorSensor.class, "cs");
        motorRightBack = hardwareMap.get(DcMotor.class, "mrb");
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront = hardwareMap.get(DcMotor.class, "mlf");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront = hardwareMap.get(DcMotor.class, "mrf");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack = hardwareMap.get(DcMotor.class, "mlb");
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLift = hardwareMap.get(DcMotor.class, "ml");
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jewel = hardwareMap.get(Servo.class, "jewels");
        left = hardwareMap.get(Servo.class, "lefts");
        right = hardwareMap.get(Servo.class, "rights");
        left1 = hardwareMap.get(Servo.class, "lefts1");
        right1 = hardwareMap.get(Servo.class, "rights1");
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart(); //Wait for the play button to be hit
        if (!opModeIsActive())
            return;
        try {
            /*RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark.equals(RelicRecoveryVuMark.UNKNOWN)) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }*/
            left.setPosition(0.5);
            right.setPosition(0.5);
            left1.setPosition(0.5);
            right1.setPosition(0.5);
            if (!opModeIsActive())
                return;
            sleep(2000);
            if (!opModeIsActive())
                return;
            left.setPosition(1);
            right.setPosition(0);
            left1.setPosition(1);
            right1.setPosition(0);
            if (!opModeIsActive())
                return;
            sleep(1000);
            if (!opModeIsActive())
                return;
            sleep(2000);
            if (!opModeIsActive())
                return;
            for (double d = 0.2; d <= 0.95; d += 0.05) {
                jewel.setPosition(d);
                sleep(500);
            }
            sleep(2000);
            if (!opModeIsActive())
                return;
            sleep(1000);
            if (!opModeIsActive())
                return;
            int visible = color();
            telemetry.addData("Color", visible);
            telemetry.update();
            if (visible >= 5) {
                turn(200);
                if (!opModeIsActive())
                    return;
                sleep(1250);
                if (!opModeIsActive())
                    return;
                jewel.setPosition(0);
                if (!opModeIsActive())
                    return;
                sleep(1250);
                if (!opModeIsActive())
                    return;
                turn(-200);
            } else if (visible <= -5) {
                turn(-200);
                if (!opModeIsActive())
                    return;
                sleep(1250);
                if (!opModeIsActive())
                    return;
                jewel.setPosition(0);
                if (!opModeIsActive())
                    return;
                sleep(1250);
                if (!opModeIsActive())
                    return;
                turn(200);

            } else {
                if (!opModeIsActive())
                    return;
                sleep(2500);
                if (!opModeIsActive())
                    return;
                jewel.setPosition(0);
                if (!opModeIsActive())
                    return;
                sleep(2500);
                if (!opModeIsActive())
                    return;
            }
        } catch (Exception e) {
        }
        if (!opModeIsActive())
            return;
        sleep(2000);
        if (!opModeIsActive())
            return;
        forward(2000);
        if (!opModeIsActive())
            return;
        sleep(2000);
        if (!opModeIsActive())
            return;
        left.setPosition(0.5);
        right.setPosition(0.5);
        left1.setPosition(0.5);
        right1.setPosition(0.5);
        if (!opModeIsActive())
            return;
        sleep(2000);
        if (!opModeIsActive())
            return;


    }

    public void forward(int i) {
        i = -i;
        motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + i);
        motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + i);
        motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() - i);
        motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() - i);
        motorRightFront.setPower(0.4);
        motorRightBack.setPower(0.4);
        motorLeftFront.setPower(0.4);
        motorLeftBack.setPower(0.4);
    }

    public void turn(int i) {
        motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + i);
        motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + i);
        motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + i);
        motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + i);
        motorRightFront.setPower(0.75);
        motorRightBack.setPower(0.75);
        motorLeftFront.setPower(0.75);
        motorLeftBack.setPower(0.75);
    }

    public void sleep(int i) {
        try {
            Thread.sleep(i);
        } catch (Exception e) {}
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