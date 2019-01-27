package org.firstinspires.AUTOMOUS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot_1")
public class Depot_1 extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private DcMotor liftMotor;
    private DcMotor intakeMotor;

    private CRServo crunchLeft;
    private CRServo crunchRight;
    private Servo outtake;

    // REV HD 40:1 Motor Specs
    double COUNTS_PER_MOTOR_REV = 2240;    // using REV HD 40:1
    double DRIVE_GEAR_REDUCTION = 0.75;    // 20 tooth to 15 tooth
    double WHEEL_DIAMETER_CM = 10.16;     // mecanum wheels
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    // Lift Motor Specs
    double COUNTS_PER_MOTOR_LIFT = 388;
    double CM_PPR_LIFT = COUNTS_PER_MOTOR_LIFT/0.8;

    //Runtime
    ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;
    public double heading;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize IMU
        imu = (BNO055IMU) hardwareMap.get("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        //Initialize Drive Motors
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // Reverse motors which were mounted upside down
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize other motors
        outtakeSlide = hardwareMap.dcMotor.get("outtakeSlide");
        intakeSlide = hardwareMap.dcMotor.get("intakeSlide");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        intakeMotor = hardwareMap.dcMotor.get("intake");

        // Reverse motors which were mounted upside down
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize servos
        crunchLeft = hardwareMap.crservo.get("crunchLeft");
        crunchRight = hardwareMap.crservo.get("crunchRight");
        outtake = hardwareMap.servo.get("outtake");

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status: ", "waiting for start command" );
            telemetry.update();
        }

        if(opModeIsActive()){
            landing(-10,1);
            landing(10,1);
            telemetry.addData("Lift Motor Encoder Counts: ", ""+ liftMotor.getCurrentPosition());
            telemetry.update();
        }

    }
    public void landing(double distanceCM, double speed) {
        int target;

        if (opModeIsActive()) {

            liftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            target = liftMotor.getCurrentPosition() + (int) (distanceCM * CM_PPR_LIFT);

            // set target position to motor
            liftMotor.setTargetPosition(target);

            // Turn on run to position
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setPower(Math.abs(speed));

            // loop while position is not reached
            while (opModeIsActive() && (liftMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("VERTICAL LINEAR SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            liftMotor.setPower(0);

            //Turn off run to position
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
