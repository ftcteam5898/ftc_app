 

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

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

@TeleOp(name="TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {

    private DcMotor motorLeftBack, motorRightBack, motorRightFront, motorLeftFront, motorLift;
    private boolean balancing = false, grasping = false;
    private Servo left, right, jewel;
    private int ticks = 0;

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1120 CPR for Andymark Motors
    private static final double     LIFT_GEAR_REDUCTION    = 0.5 ;     // Gearing Up by 2
    private static final double     GEAR_DIAMETER_INCHES   = 2.58 ;     // For figuring circumference
    private static final double     LIFT_COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION) /
            (GEAR_DIAMETER_INCHES * 3.1415);
    private static double cPos = 0.0;


    @Override
    public void runOpMode() {

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
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jewel = hardwareMap.get(Servo.class, "jewels");
        left = hardwareMap.get(Servo.class, "lefts");
        right = hardwareMap.get(Servo.class, "rights");
        

        //Wait for the play button to be hit
        waitForStart();

        left.setPosition(0.5);
        right.setPosition(0.5);
        //BEGIN
        while (opModeIsActive()) {
            //Reassign balancing variable
            jewel.setPosition(0);
            if (gamepad1.a) {
                balancing = !balancing;
            }
            if (gamepad1.x /* && motorLift.getCurrentPosition()>-150)*/) {
                motorLift.setPower(1);
            } else if (gamepad1.y /* && motorLift.getCurrentPosition()<150)*/) {
                motorLift.setPower(-1);
            } else {
                motorLift.setPower(0);
            }
            if (gamepad1.left_bumper) {
                left.setPosition(0.5);
                right.setPosition(0.5);
            }
            if (gamepad1.right_bumper) {
                left.setPosition(0.8);
                right.setPosition(0.2);
            }
            //Check if we are in the balancing process
            if (balancing) {

                //Begin balancing - Remember we are working off of the last checks information
                telemetry.addData("Mode", "Gentle Balancing");
                telemetry.addData("Lift Position",  "%7d",
                          motorLift.getCurrentPosition());

                //Read controller input
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
                motorRightFront.setPower(0.5*FrontRight);
                motorLeftFront.setPower(0.5*FrontLeft);
                motorLeftBack.setPower(0.5*BackLeft);
                motorRightBack.setPower(0.5*BackRight);
                telemetry.update();

            //Do normal control scenario
            } else {

                //Regular TeleOp Telemetry
                telemetry.addData("Mode", "Controllers Active");
                telemetry.addData("Lift Position",  "%7d",
                          motorLift.getCurrentPosition());


                //Read controller input
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
    }

}
