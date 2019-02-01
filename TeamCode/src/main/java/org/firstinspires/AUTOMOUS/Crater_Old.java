///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.AUTOMOUS;
//
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
//
//
//
//@Autonomous(name = "Crater_1", group = "Autonomous")
//@Disabled
//public class Crater_1 extends LinearOpMode {
//
//    private DcMotor driveFrontLeft;
//    private DcMotor driveFrontRight;
//    private DcMotor driveBackLeft;
//    private DcMotor driveBackRight;
//
//    private DcMotor verticalLinearSlide;
//
//    private Servo marker;
//    private Servo cheeze;
//
//    // MAIN ROBOT
//    double robotRadius = 31.51267873;
//    double COUNTS_PER_MOTOR = 2240;    // using REV HD 40:1
//    double DRIVE_GEAR_REDUCTION = 26/15;    // 26 tooth to 15 tooth
//    double WHEEL_DIAMETER_CM = 10.16;     // mecanum wheels
//
////    // TEST ROBOT
////    double robotRadius = 16.265;
////    double COUNTS_PER_MOTOR = 1680;// using Andymark 40:1
////    double DRIVE_GEAR_REDUCTION = 0.75;// 40 tooth to 20 tooth
////    double WHEEL_DIAMETER_CM = 7.62;// omni wheels
//
//    double COUNTS_PER_CM = ((COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;
//
//    //Runtime
//    ElapsedTime runtime = new ElapsedTime();
//
//    // The IMU sensor object
//    BNO055IMU imu;
//    public double heading;
//
//    private GoldAlignDetector detector;
//    private String goldMineralPosition = " ";
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
//
////        // Initialize IMU
////        imu = (BNO055IMU) hardwareMap.get("imu");
////        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
////        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
////        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
////        imu.initialize(parameters);
////
////        // Get starting heading pitch and roll
////        heading = getCurrentHeading();
//
//        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
//        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
//        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
//        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
//
//        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
//        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        verticalLinearSlide = hardwareMap.dcMotor.get("verticalLinearSlide");
//        verticalLinearSlide.setDirection(DcMotor.Direction.REVERSE);
//
//        marker = hardwareMap.servo.get("marker");
//        cheeze = hardwareMap.servo.get("cheeze");
//
//            if (detector.getXPosition() > 300) {
//                goldMineralPosition = "RIGHT";
//            } else if (detector.getXPosition() <= 300 && detector.getXPosition() != 0) {
//                goldMineralPosition = "MIDDLE";
//            }else {
//                goldMineralPosition = "LEFT";
//            }
//
//        telemetry.addData(" GOLD MINERAL POSITION ", goldMineralPosition + " POSITION: " + detector.getXPosition());
//        telemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//
////            if (detector.isFound() & opModeIsActive()) {
////                if (detector.getXPosition() > 340) {
////                    goldMineralPosition = "RIGHT";
////                    telemetrySender("GOLD MINERAL POSITION", goldMineralPosition, "YES");
////                } else if (detector.getXPosition() < 240) {
////                    goldMineralPosition = "LEFT";
////                    telemetrySender("GOLD MINERAL POSITION", goldMineralPosition, "YES");
////                } else if (240 < detector.getXPosition() & 340 > detector.getXPosition()) {
////                    goldMineralPosition = "MIDDLE";
////                    telemetrySender("GOLD MINERAL POSITION", goldMineralPosition, "YES");
////                }
////            } else {
////                telemetrySender("MINERAL NOT FOUND", "", "YES");
////            }
//
//            marker.setPosition(0);
//            cheeze.setPosition(0.5);
//
//            if (opModeIsActive()) {
//                if (detector.getXPosition() > 300) {
//                    goldMineralPosition = "RIGHT";
//                } else if (detector.getXPosition() <= 300 && detector.getXPosition() != 0) {
//                    goldMineralPosition = "MIDDLE";
//                } else {
//                    goldMineralPosition = "LEFT";
//                }
//            }
//            telemetry.addData(" GOLD MINERAL POSITION ", goldMineralPosition);
//            telemetry.update();
//
//            sleep(1000);
//
//            landing(5.1,1, 7,50);
//            straightDriveEncoder(0.25,-5,0.5,50);
//            strafeDriveEncoder(0.25,10,1.0,50,"LEFT");
//            straightDriveEncoder(0.5,5,0.5,50);
//            strafeDriveEncoder(0.5,10,0.5,50,"RIGHT");
//
//
//            switch (goldMineralPosition) {
//                case "LEFT": // Gold mineral in right position
//                    turnEncoder(0.5, 35, "CC",0.5);
//                    straightDriveEncoder(0.5, 60 , 2, 50);
//                    straightDriveEncoder(0.5, -59 , 2, 50);
//                    turnEncoder(0.5, 10, "CC",0.5);
//                    break;
//                case "RIGHT":// Gold mineral in left position
//                    turnEncoder(0.5, 40, "C",0.5);
//                    straightDriveEncoder(0.5, 60 , 2, 50);
//                    straightDriveEncoder(0.5, -59 , 2, 50);
//                    turnEncoder(0.5, 70, "CC",0.5);
//                    break;
//                case "MIDDLE": // Gold mineral in middle position
//                    straightDriveEncoder(0.5, 55 , 2, 50);
//                    straightDriveEncoder(0.5, -53 , 2, 50);
//                    turnEncoder(0.5,45,"CC",0.5);
//                    break;
//                default:
//                    telemetrySender("MINERAL NOT FOUND", "", "YES");
//                    break;
//            }
//            straightDriveEncoder(0.5,100,1.5,50);
//            turnEncoder(0.5,90,"CC",1.0);
//            strafeDriveEncoder(0.5,30,0.5,50,"RIGHT");
//            strafeDriveEncoder(0.5,10,0.5,50,"LEFT");
//            straightDriveEncoder(1.0,180,2.0,50);
//            strafeDriveEncoder(0.5,20,0.5,50,"LEFT");
//            dropMarker(500);
//            straightDriveEncoder(1.0,-10,2.0,50);
//            strafeDriveEncoder(0.5,30,0.5,50,"RIGHT");
//            turnEncoder(0.5,180,"CC",2.0);
//            strafeDriveEncoder(0.5,10,0.5,50,"LEFT");
//            cheeze();
//            straightDriveEncoder(1.0,160,2.0,50);
//
//        }
//        idle();
//    }
//
//    public void strafeDriveEncoder(double speed, double distance, double totalRuntime, int sleep, String direction) {
//        int frontLeftTarget = 0;
//        int backLeftTarget = 0;
//        int frontRightTarget = 0;
//        int backRightTarget = 0;
//
//        switch (direction) {
//            case "LEFT":
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5 * -1);
//                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5);
//                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5);
//                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5 * -1);
//                break;
//            case "RIGHT":
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5);
//                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5 * -1);
//                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5 * -1);
//                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM * 1.5);
//                break;
//        }
//        if (opModeIsActive()) {
//
//            // set target position to each motor
//            driveFrontLeft.setTargetPosition(frontLeftTarget);
//            driveFrontRight.setTargetPosition(frontRightTarget);
//            driveBackLeft.setTargetPosition(backLeftTarget);
//            driveBackRight.setTargetPosition(backRightTarget);
//
//            // Turn on run to position
//            motorStateSetter("RUN_TO_POSITION");
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//
//            driveFrontLeft.setPower(Math.abs(speed));
//            driveFrontRight.setPower(Math.abs(speed));
//            driveBackLeft.setPower(Math.abs(speed));
//            driveBackRight.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < totalRuntime) &&
//                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
//                telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
//                telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
//                telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            driveFrontLeft.setPower(0);
//            driveFrontRight.setPower(0);
//            driveBackLeft.setPower(0);
//            driveBackRight.setPower(0);
//
//            //Turn off run to position
//            motorStateSetter("RUN_USING_ENCODER");
//
//            //Message
//            telemetrySender("PATH COMPLETED ", "SLEEP STATE", "YES");
//
//            //SLEEP
//            sleep(sleep);
//
//        }
//
//    }
//
//    public void straightDriveEncoder(double speed, double distance, double totalRuntime, long sleep) {
//        int frontLeftTarget;
//        int backLeftTarget;
//        int frontRightTarget;
//        int backRightTarget;
//
//        if (opModeIsActive()) {
//
//            telemetrySender("RESET ENCODERS", " !!!!!!! ", "YES");
//
//            motorStateSetter("RESET_ENCODERS");
//
//            // Determine new target position, and pass to motor controller
//            frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//            frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//            backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//            backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//
//            // set target position to each motor
//            driveFrontLeft.setTargetPosition(frontLeftTarget);
//            driveFrontRight.setTargetPosition(frontRightTarget);
//            driveBackLeft.setTargetPosition(backLeftTarget);
//            driveBackRight.setTargetPosition(backRightTarget);
//
//            // Turn on run to position
//            motorStateSetter("RUN_TO_POSITION");
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//
//            driveFrontLeft.setPower(Math.abs(speed));
//            driveFrontRight.setPower(Math.abs(speed));
//            driveBackLeft.setPower(Math.abs(speed));
//            driveBackRight.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < totalRuntime) &&
//                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
//                telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
//                telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
//                telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            driveFrontLeft.setPower(0);
//            driveFrontRight.setPower(0);
//            driveBackLeft.setPower(0);
//            driveBackRight.setPower(0);
//
//            //Turn off run to position
//            motorStateSetter("RUN_USING_ENCODER");
//
//            //Message
//            telemetrySender("PATH COMPLETED ", "SLEEP STATE", "YES");
//
//            //SLEEP
//            sleep(sleep);
//        }
//    }
//
//    public void turnEncoder(double speed, double turnDegrees, String direction, double totalRuntime) {
//        double distance = robotRadius * (((turnDegrees) * (Math.PI)) / (180)); // Using arc length formula
//        int frontLeftTarget = 0;
//        int backLeftTarget = 0;
//        int frontRightTarget = 0;
//        int backRightTarget = 0;
//
//        telemetrySender("RESET ENCODERS", " !!!!!!! ", "YES");
//
//        motorStateSetter("RESET_ENCODERS");
//
//        //double headingStart = getCurrentHeading();
//
//        // Display it for the driver.
//        telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
//        telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
//        telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
//        telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
//        telemetry.update();
//
//        switch (direction) {
//            case "C":
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM);
//                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM);
//                break;
//            case "CC":
//                // Determine new target position, and pass to motor controller
//                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM);
//                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM);
//                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
//                break;
//        }
//
//        if (opModeIsActive()) {
//
//            // set target position to each motor
//            driveFrontLeft.setTargetPosition(frontLeftTarget);
//            driveFrontRight.setTargetPosition(frontRightTarget);
//            driveBackLeft.setTargetPosition(backLeftTarget);
//            driveBackRight.setTargetPosition(backRightTarget);
//
//            // Turn on run to position
//            motorStateSetter("RUN_TO_POSITION");
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//
//            driveFrontLeft.setPower(speed);
//            driveFrontRight.setPower(speed);
//            driveBackLeft.setPower(speed);
//            driveBackRight.setPower(speed);
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < totalRuntime) &&
//                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {
//
//                // Display it for the driver.
//                //telemetrySender("DEGREES", "" + getCurrentHeading(), "YES");
//            }
//
//            // Stop all motion;
//            driveFrontLeft.setPower(0);
//            driveFrontRight.setPower(0);
//            driveBackLeft.setPower(0);
//            driveBackRight.setPower(0);
//
//            //Turn off run to position
//            motorStateSetter("RUN_USING_ENCODER");
//
//            //Message
//            telemetrySender("ROUGH COMPLETED ", "SLEEP STATE", "YES");
//
//            //SLEEP
//            sleep(500);
//        }
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        //telemetrySender("DEGREES CURRENT: ", "" + getCurrentHeading(), "");
//        //telemetrySender("DEGREES FINAL: ", "" + (getCurrentHeading() + headingStart), "");
//    }
//
//    public void turnAlignMineral(double speed, double totalRuntime, long sleep) {
//        if (detector.getXPosition() > 340) { // turn right
//
//            driveFrontLeft.setPower(speed);
//            driveFrontRight.setPower(speed * -1);
//            driveBackLeft.setPower(speed);
//            driveBackRight.setPower(speed * -1);
//
//            runtime.reset();
//
//            while (opModeIsActive() &&
//                    (runtime.seconds() < totalRuntime) &&
//                    (detector.getXPosition() > 290)) {
//
//                telemetry.addData("X - POSITION", ": " + detector.getXPosition());
//                telemetry.update();
//                sleep(50);
//            }
//
//            driveFrontLeft.setPower(0);
//            driveFrontRight.setPower(0);
//            driveBackLeft.setPower(0);
//            driveBackRight.setPower(0);
//        } else if (detector.getXPosition() < 240) { // turn left
//
//            driveFrontLeft.setPower(speed * -1);
//            driveFrontRight.setPower(speed);
//            driveBackLeft.setPower(speed * -1);
//            driveBackRight.setPower(speed);
//
//            runtime.reset();
//
//            while (opModeIsActive() &&
//                    (runtime.seconds() < totalRuntime) &&
//                    (detector.getXPosition() < 290)) {
//
//                telemetry.addData("X - POSITION", ": " + detector.getXPosition());
//                telemetry.update();
//                sleep(50);
//            }
//
//            driveFrontLeft.setPower(0);
//            driveFrontRight.setPower(0);
//            driveBackLeft.setPower(0);
//            driveBackRight.setPower(0);
//        }
//
//    }
//
//    public void motorStateSetter(String motorState) {
//
//        switch (motorState) {
//            case "RUN_TO_POSITION":
//                driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                break;
//            case "RUN_USING_ENCODER":
//                driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                break;
//            case "RUN_WITHOUT_ENCODER":
//                driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                break;
//            case "RESET_ENCODERS":
//                driveFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//                driveFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//                driveBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//                driveBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//                break;
//            case "STOP_AND_RESET_ENCODER":
//                driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                break;
//            default:
//                telemetrySender("ERROR", ": NOT VALID MOTOR STATE", "YES");
//                break;
//        }
//    }
//
//    public void telemetrySender(String identifier, String data, String clear) {
//
//        if (clear == "YES") {
//            telemetry.clearAll();
//            telemetry.update();
//        }
//
//        telemetry.addData(identifier, data);
//        telemetry.update();
//    }
//
////    public double getCurrentHeading() {
////        heading = 0.0;
////        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle * -1;
////        if (heading < 0) {
////            heading = heading + 360;
////        }
////        return heading;
////    }
//
//    public void landing(double distance, double speed, double totalRuntime, long sleep) {
//        int target;
//
//        if (opModeIsActive()) {
//
//            telemetrySender("RESET ENCODERS", " !!!!!!! ", "YES");
//
//            verticalLinearSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//            // Determine new target position, and pass to motor controller
//            target = verticalLinearSlide.getCurrentPosition() + (int) (distance * COUNTS_PER_MOTOR);
//
//            // set target position to each motor
//            verticalLinearSlide.setTargetPosition(target);
//
//            // Turn on run to position
//            verticalLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//
//            verticalLinearSlide.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < totalRuntime) &&
//                    (verticalLinearSlide.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("VERTICAL LINEAR SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, verticalLinearSlide.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            verticalLinearSlide.setPower(0);
//
//            //Turn off run to position
//            verticalLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //Message
//            telemetrySender("LANDING COMPLETED ", "SLEEP STATE", "YES");
//
//            //SLEEP
//            sleep(sleep);
//        }
//    }
//
//    public void dropMarker(long sleep){
//        marker.setPosition(0.65);
//        sleep(sleep);
//        marker.setPosition(0.0);
//    }
//
//    public void cheeze(){
//        cheeze.setPosition(0.2);
//    }
//
//    // Turn using IMU
//    // Strafe using distance sensor
//    // Drive forward using distance sensor
//    // Drive to line using color sensor
//
//}
