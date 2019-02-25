package org.firstinspires.AUTOMOUS;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "RESET")
public class RESET extends LinearOpMode {

    private DcMotor liftMotor;
    private Servo outtake;
    private Servo marker;
    private DcMotor outtakeSlide;

    // Lift Motor Specs
    double COUNTS_PER_MOTOR_LIFT = 388;
    double CM_PPR_LIFT = COUNTS_PER_MOTOR_LIFT / 0.8;
    // REV HD 40:1 Motor Specs
    double COUNTS_PER_MOTOR_REV = 2240;    // using REV HD 40:1
    double DRIVE_GEAR_REDUCTION = 0.75;    // 20 tooth to 15 tooth
    double WHEEL_DIAMETER_CM = 10.16;     // mecanum wheels
    double TUNING_DRIVE = 1.1;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    // Intake Motor Specs
    double TUNING_INTAKE = 10;
    double COUNTS_PER_CM_INTAKE = ((COUNTS_PER_CM_REV * TUNING_INTAKE));


    @Override
    public void runOpMode() throws InterruptedException {

        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        outtake = hardwareMap.servo.get("outtake");
        outtakeSlide = hardwareMap.dcMotor.get("outtakeSlide");
        marker = hardwareMap.servo.get("marker");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.setPosition(0.45);
        marker.setPosition(0.62);
        outtakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status: ", "waiting for start command");
            telemetry.update();
        }

        while (opModeIsActive()) {

            if(gamepad1.x){
                landing(-17, 1.0);
            }
            if(gamepad1.y){
                outtake.setPosition(0.45);
                outtakeSlideEncoder(1.0,-26);
            }



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

    // NOT GETTING ANY ENCODER READINGS
    public void outtakeSlideEncoder(double speed, double distanceCM) {
        int target;

        if (opModeIsActive()) {

            outtakeSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            target = outtakeSlide.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_INTAKE);

            // set target position to each motor
            outtakeSlide.setTargetPosition(target);

            // Turn on run to position
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            outtakeSlide.setPower(Math.abs(speed));

            // Display it for the driver.
            telemetry.addData("INTAKE SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, outtakeSlide.getCurrentPosition());
            telemetry.update();

            while (opModeIsActive() &&
                    (outtakeSlide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("INTAKE SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, outtakeSlide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            outtakeSlide.setPower(0);

            //Turn off run to position
            outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
