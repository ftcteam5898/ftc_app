package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="BP1", group="Autonomous")
public class BP1 extends LinearOpMode {

    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront, motorLift;
    private Servo left, right, jewel;
    private ColorSensor cs;

    @Override
    public void runOpMode() {

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
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        jewel = hardwareMap.get(Servo.class, "jewels");
        left = hardwareMap.get(Servo.class, "lefts");
        right = hardwareMap.get(Servo.class, "rights");

        //Wait for the play button to be hit
        waitForStart();

        try {
            if (opModeIsActive()) {

                cs.enableLed(true);

                //Grasp
                left.setPosition(1);
                right.setPosition(0);

                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                }

                motorLift.setPower(-1);
                try {
                    Thread.sleep(750);
                } catch (Exception e) {
                }
                motorLift.setPower(0);


                //extend jewel displacer
                jewel.setPosition(0);
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
                int rminusbsum = 0;
                for (int i = 0; i < 10; i++) {
                    rminusbsum += (cs.red() - cs.blue());
                    telemetry.addData("Color", cs.red() + " " + cs.blue());
                    telemetry.update();
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                }
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
                //Loop until we know what we are looking at
                //relicTrackables.activate();
                //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                //telemetry.addData("VuMark", "%s visible", vuMark);
                //Hit the correct jewel
                if (rminusbsum <= 0) {
                    motorLeftBack.setPower(1);
                    motorLeftFront.setPower(1);
                    motorRightBack.setPower(-1);
                    motorRightFront.setPower(-1);
                } else {
                    motorLeftBack.setPower(-1);
                    motorLeftFront.setPower(-1);
                    motorRightBack.setPower(1);
                    motorRightFront.setPower(1);
                }
                try {
                    Thread.sleep(250);
                } catch (Exception e) {
                }
                telemetry.addData("Hi", "Hi");
                motorLeftBack.setPower(0);
                motorLeftFront.setPower(0);
                motorRightBack.setPower(0);
                motorRightFront.setPower(0);
                if (rminusbsum <= 0) {
                    motorLeftBack.setPower(-1);
                    motorLeftFront.setPower(-1);
                    motorRightBack.setPower(1);
                    motorRightFront.setPower(1);
                } else {
                    motorLeftBack.setPower(1);
                    motorLeftFront.setPower(1);
                    motorRightBack.setPower(-1);
                    motorRightFront.setPower(-1);
                }
                try {
                    Thread.sleep(200);
                } catch (Exception e) {
                }
                motorLeftBack.setPower(0);
                motorLeftFront.setPower(0);
                motorRightBack.setPower(0);
                motorRightFront.setPower(0);
                jewel.setPosition(1);
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
                cs.enableLed(false);
            }
        } catch (Exception e) {
        }

    }

    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
