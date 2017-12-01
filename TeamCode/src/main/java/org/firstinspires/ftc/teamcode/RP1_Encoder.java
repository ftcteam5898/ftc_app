import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="RP1_Encoder", group="Autonomous")
public class RP1_Encoder extends LinearOpMode {

    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront, motorLift;
    private Servo left, right, jewel;
    private ColorSensor cs;
    
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Andymark Motor CPR
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No gear reduction
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

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
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        jewel = hardwareMap.get(Servo.class, "jewels");
        left = hardwareMap.get(Servo.class, "lefts");
        right = hardwareMap.get(Servo.class, "rights");
        
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wait for the play button to be hit
        waitForStart();

        try {
            if (opModeIsActive()) {

                cs.enableLed(true);

                //Grasp
                left.setPosition(.8);
                right.setPosition(.2);
                sleep(1000); //wait for servo

                raise(1, 3); // raise lift at full power 3 inches
                
                //extend jewel displacer
                jewel.setPosition(0);
                sleep(500);
                
                
                int rminusbsum = 0;
                for (int i = 0; i < 10; i++) {
                    rminusbsum += (cs.red() - cs.blue());
                    sleep(10);
                }
                
                //Loop until we know what we are looking at
                //relicTrackables.activate();
                //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                //telemetry.addData("VuMark", "%s visible", vuMark);
                
                //Hit the correct jewel
                if (rminusbsum > 0) {
                    telemetry.addData("Red color found");
                    telemetry.update();
                    driveForward(.5, 5); //half speed for 5 inches
                    driveBackward(.5, 5);
                } else {
                    telemetry.addData("Blue color found");
                    telemetry.update();
                    driveBackward(.5, 5);
                    driveForward(.5, 5);
                }
                
                //return jewel arm & turn off color sensor
                jewel.setPosition(1); 
                sleep(500); //wait for servo
                cs.enableLed(false);
                
                // driving towards glyph area
                driveForward(.5, 40); //half speed for 40 inches
                rotateRight(.3, 10); //30% speed for 10 inches
                lower(1, 7); //lower lift at full speed 7 inches
                //driveForward(.25, 4);
                
                //Let Go of glyph
                left.setPosition(.5);
                right.setPosition(.5);
                //driveBackward(.25, 4);

            }
        } catch (Exception e) {
        }

    }

    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    
    public void rotateRight(double speed, double inches) {
        int mlfNewTarget;
        int mlbNewTarget;
        int mrfNewTarget;
        int mrbNewTarget;
        if (opModeIsActive()) {
            // Determine new target position
            mlfNewTarget = motorLeftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            mlbNewTarget = motorLeftBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            mrfNewTarget = motorRightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            mrbNewTarget = motorRightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            
            // Turn on RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // start motion
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));
            
            while (opModeIsActive() &&
                   (motorLeftFront.isBusy() && motorLeftBack.isBusy()
                   && motorRightFront.isBusy() && motorRightBack.isBusy())) {
                        // Display info for the driver.
                        telemetry.addData("Running to %7d", mlfNewTarget);
                        telemetry.addData("Running at %7d",
                                                    motorLeftFront.getCurrentPosition());
                        telemetry.update();
            }
            
            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void rotateLeft(double speed, double inches) {
        int mlfNewTarget;
        int mlbNewTarget;
        int mrfNewTarget;
        int mrbNewTarget;
        if (opModeIsActive()) {
            // Determine new target position
            mlfNewTarget = motorLeftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            mlbNewTarget = motorLeftBack.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            mrfNewTarget = motorRightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            mrbNewTarget = motorRightBack.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            
            // Turn on RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // start motion
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));
            
            while (opModeIsActive() &&
                   (motorLeftFront.isBusy() && motorLeftBack.isBusy()
                   && motorRightFront.isBusy() && motorRightBack.isBusy())) {
                        // Display info for the driver.
                        telemetry.addData("Running to %7d", mlfNewTarget);
                        telemetry.addData("Running at %7d",
                                                    motorLeftFront.getCurrentPosition());
                        telemetry.update();
            }
            
            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void driveForward(double speed, double inches) {
        int mlfNewTarget;
        int mlbNewTarget;
        int mrfNewTarget;
        int mrbNewTarget;
        if (opModeIsActive()) {
            // Determine new target position
            mlfNewTarget = motorLeftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            mlbNewTarget = motorLeftBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            mrfNewTarget = motorRightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            mrbNewTarget = motorRightBack.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            
            // Turn on RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // start motion
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));
            
            while (opModeIsActive() &&
                   (motorLeftFront.isBusy() && motorLeftBack.isBusy()
                   && motorRightFront.isBusy() && motorRightBack.isBusy())) {
                        // Display info for the driver.
                        telemetry.addData("Running to %7d", mlfNewTarget);
                        telemetry.addData("Running at %7d",
                                                    motorLeftFront.getCurrentPosition());
                        telemetry.update();
            }
            
            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveBackward(double speed, double inches) {
        int mlfNewTarget;
        int mlbNewTarget;
        int mrfNewTarget;
        int mrbNewTarget;
        if (opModeIsActive()) {
            // Determine new target position
            mlfNewTarget = motorLeftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            mlbNewTarget = motorLeftBack.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            mrfNewTarget = motorRightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            mrbNewTarget = motorRightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            
            // Turn on RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // start motion
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));
            
            while (opModeIsActive() &&
                   (motorLeftFront.isBusy() && motorLeftBack.isBusy()
                   && motorRightFront.isBusy() && motorRightBack.isBusy())) {
                        // Display info for the driver.
                        telemetry.addData("Running to %7d", mlfNewTarget);
                        telemetry.addData("Running at %7d",
                                                    motorLeftFront.getCurrentPosition());
                        telemetry.update();
            }
            
            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void raise(double speed, double inches) {
        double     COUNTS_PER_MOTOR_REV    = 1120 ;     // Andymark Motor CPR
        double     LIFT_GEAR_REDUCTION     = 0.5 ;      // Geared up by 2:1
        double     GEAR_DIAMETER_INCHES    = 2.58 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
                                                      
        int motorNewTarget;
        if (opModeIsActive()) {
            // Determine new target position
            motorNewTarget = motorLift.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            // Turn on RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion
            motorLeftFront.setPower(Math.abs(speed));
            while (opModeIsActive() && motorLift.isBusy()) {
                // Display info for the driver.
                telemetry.addData("Lifting to %7d", motorNewTarget);
                telemetry.addData("Lifting at %7d", motorLift.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            motorLift.setPower(0);
            // Turn off RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void lower(double speed, double inches) {
        double     COUNTS_PER_MOTOR_REV    = 1120 ;     // Andymark Motor CPR
        double     LIFT_GEAR_REDUCTION     = 0.5 ;      // Geared up by 2:1
        double     GEAR_DIAMETER_INCHES    = 2.58 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
                                                      
        int motorNewTarget;
        if (opModeIsActive()) {
            // Determine new target position
            motorNewTarget = motorLift.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            // Turn on RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion
            motorLeftFront.setPower(Math.abs(speed));
            while (opModeIsActive() && motorLift.isBusy()) {
                // Display info for the driver.
                telemetry.addData("Lifting to %7d", motorNewTarget);
                telemetry.addData("Lifting at %7d", motorLift.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            motorLift.setPower(0);
            // Turn off RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
