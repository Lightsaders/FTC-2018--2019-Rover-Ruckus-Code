package org.firstinspires.AUTOMOUS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "DEPOT_OPPOSITE_CRATER")
public class DEPOT_OPPOSITE_CRATER extends LinearOpMode {

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
    double TUNING_DRIVE = 1.1;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    // Lift Motor Specs
    double COUNTS_PER_MOTOR_LIFT = 388;
    double CM_PPR_LIFT = COUNTS_PER_MOTOR_LIFT / 0.8;

    // Intake Motor Specs
    double TUNING_INTAKE = 5;
    double COUNTS_PER_CM_INTAKE = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_INTAKE));

    //Runtime
    ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;
    public double heading;

    // Tensorflow
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "ARCIeAv/////AAABmc7rRZU9AUZJjXc1QoY3z+94AeSN1dBKD0EWtrasT+QqXPat3vIFa09vi3b9xjMmmOi65gII0IG3WDmxFxffEg8sU8ZIihGqQnYoDRN1ho6p5pKIYEDfWGURt4ykZ5US5w5cCEBcssxAI5zWpVvpYBm9uDGO0EiC4rr0jeEJ9jZzCS7oYLSe1kbF5J8UT89edXK1YfLS6tPF59LgBOWgJSHSDmHJcGdIGXY2oejpN+vKNI19bdtSFJ+tyKCS8EZdGomuClneCQRLCn3rOthemk53T4gGErz/Ro/3zLmzbAZ2PCMMXTMW8RoTVIDYx05mnfpzNyf7Ve8GyuY8YTCqWeANJPh7oIyzuXrraup7MSWy";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String position;


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
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize servos
        crunchLeft = hardwareMap.crservo.get("crunchLeft");
        crunchRight = hardwareMap.crservo.get("crunchRight");
        outtake = hardwareMap.servo.get("outtake");

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status: ", "waiting for start command");
            telemetry.update();
        }

        if (opModeIsActive()) {

            String mineral = " ";

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        double goldMineralConfidence = 0.0;
                        double silverMineralConfidence = 0.0;
                        double start = getRuntime();
                        while (opModeIsActive() && getRuntime()-start < 5 && mineral == " ") {
                            for (Recognition recognition : updatedRecognitions) {

                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && Math.abs(recognition.getLeft() - recognition.getRight()) > 100) {
                                    goldMineralConfidence = recognition.getConfidence();
                                }
                                if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) && Math.abs(recognition.getLeft() - recognition.getRight()) > 100) {
                                    silverMineralConfidence = recognition.getConfidence();
                                }

                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && Math.abs(recognition.getLeft() - recognition.getRight()) > 100) {
                                    goldMineralX = (int) recognition.getBottom();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getBottom();
                                } else {
                                    silverMineral2X = (int) recognition.getBottom();
                                }
                            }

                            if (goldMineralX > 700 && silverMineral1X < 700 && goldMineralConfidence > silverMineralConfidence) {
                                mineral = "C";
                                telemetry.addData("Mineral POSITION: ", mineral);
                                telemetry.update();
                            } else if (goldMineralX < 700 && silverMineral1X > 700 && goldMineralConfidence > silverMineralConfidence) {
                                mineral = "L";
                                telemetry.addData("Mineral POSITION: ", mineral);
                                telemetry.update();
                            } else if (silverMineral1X != -1 && silverMineral2X != -1) {
                                mineral = "R";
                                telemetry.addData("Mineral POSITION: ", mineral);
                                telemetry.update();
                            }

                        }
                        telemetry.update();
                    }
                }

            if (tfod != null) {
                tfod.shutdown();
            }

            // LAND
            landing(17, 1.0);

            //DELATCH
            strafeDriveEncoder(0.5, 4, "RIGHT");
            straightDriveEncoder(0.5, 5);
            strafeDriveEncoder(0.5, 4, "RIGHT");
            straightDriveEncoder(0.5, -5);

            // TURN TO MINERAL
            switch(mineral){
                case"L":
                    turnEncoder(0.3,  45, "C");
                    straightDriveEncoder(0.5,68);
                    turnEncoder(0.3,  75, "C");
                    straightDriveEncoder(0.5,68);
                    // DROP MINERAL
                    turnEncoder(0.5,165,"CC");
                    strafeDriveEncoder(0.5,10,"RIGHT");
                    strafeDriveEncoder(0.5,5,"LEFT");
                    straightDriveEncoder(0.5,134);
                    break;
                case"R":
                    turnEncoder(0.3,  135, "C");
                    straightDriveEncoder(0.5,72);
                    turnEncoder(0.3,  75, "CC");
                    straightDriveEncoder(0.5,65);
                    //DROP MINERAL
                    turnEncoder(0.3,105,"CC");
                    strafeDriveEncoder(0.5,30,"RIGHT");
                    strafeDriveEncoder(0.5,5,"LEFT");
                    straightDriveEncoder(0.5,154);
                    break;
                case"C":
                    turnEncoder(0.3,  90, "C");//
                    straightDriveEncoder(0.5,105);
                    straightDriveEncoder(0.5,-15);
                    turnEncoder(0.5,135,"CC");
                    // DROP MINERAL
                    strafeDriveEncoder(0.5,33,"RIGHT");
                    strafeDriveEncoder(0.5,5,"LEFT");
                    straightDriveEncoder(0.5,140);
                    break;
            }

            double run1 = getRuntime() + 0.75;
            while (run1 > getRuntime()) {// crunch DOWN
                crunchLeft.setPower(-1);
                crunchRight.setPower(1);
            }
            crunchLeft.setPower(0);
            crunchRight.setPower(0);

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

    // NOT PERFECT BUT SHOULD WORK, MAY NEED TO REVISIT
//    public void IMUturn(double degree, String direction, double speed, double tolerance) {
//
//        double initialHeading;
//        double finalHeading;
//
//        switch (direction) {
//            case "C":// clockwise
//
//                initialHeading = getCurrentHeading();
//                finalHeading = initialHeading + degree;
//                if (finalHeading > 360) {
//                    finalHeading = finalHeading - 360;
//                }
//
//                while (opModeIsActive() && (int) (getCurrentHeading()) < (int) (finalHeading + tolerance)) {
//                    driveFrontLeft.setPower(speed);
//                    driveFrontRight.setPower(speed * -1);
//                    driveBackLeft.setPower(speed);
//                    driveBackRight.setPower(speed * -1);
//                    telemetry.addData("1CURRENT HEADING: ", "" + getCurrentHeading() + " " + finalHeading);
//                    telemetry.update();
//                }
//
//                driveFrontLeft.setPower(0);
//                driveFrontRight.setPower(0);
//                driveBackLeft.setPower(0);
//                driveBackRight.setPower(0);
//
//                break;
//            case "CC":// counterclockwise
//
//                initialHeading = getCurrentHeading();
//                finalHeading = initialHeading - degree;
//                if (finalHeading < 0) {
//                    finalHeading = finalHeading + 360;
//                }
//
//                while (opModeIsActive() && (int) (getCurrentHeading()) > (int) (finalHeading - tolerance)) {
//                    driveFrontLeft.setPower(speed * -1);
//                    driveFrontRight.setPower(speed);
//                    driveBackLeft.setPower(speed * -1);
//                    driveBackRight.setPower(speed);
//                    telemetry.addData("2CURRENT HEADING: ", "" + getCurrentHeading() + " " + finalHeading);
//                    telemetry.update();
//                }
//
//                driveFrontLeft.setPower(0);
//                driveFrontRight.setPower(0);
//                driveBackLeft.setPower(0);
//                driveBackRight.setPower(0);
//
//                break;
//        }
//    }

    public void turnEncoder(double speed, double turnDegrees, String direction) {
        double distance = ROBOT_RADIUS_CM * (((turnDegrees) * (Math.PI)) / (180)); // Using arc length formula
        int frontLeftTarget = 0;
        int backLeftTarget = 0;
        int frontRightTarget = 0;
        int backRightTarget = 0;

        //RESET ENCODERS
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch (direction) {
            case "C":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                break;
            case "CC":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                break;
        }

        if (opModeIsActive()) {

            // set target position to each motor
            driveFrontLeft.setTargetPosition(frontLeftTarget);
            driveFrontRight.setTargetPosition(frontRightTarget);
            driveBackLeft.setTargetPosition(backLeftTarget);
            driveBackRight.setTargetPosition(backRightTarget);

            // Turn on run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            driveFrontLeft.setPower(speed);
            driveFrontRight.setPower(speed);
            driveBackLeft.setPower(speed);
            driveBackRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                // Display it for the driver.
                //telemetrySender("DEGREES", "" + getCurrentHeading(), "YES");
            }

            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            //Turn off run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
        }

        // reset the timeout time and start motion.
        runtime.reset();
        //telemetrySender("DEGREES CURRENT: ", "" + getCurrentHeading(), "");
        //telemetrySender("DEGREES FINAL: ", "" + (getCurrentHeading() + headingStart), "");
    }
    
    public double getCurrentHeading() {
        heading = 0.0;
        heading = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle * -1);
        if (heading < 0) {
            heading = heading + 360;
        }
        return heading;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void straightDriveEncoder(double speed, double distanceCM) {
        int frontLeftTarget;
        int backLeftTarget;
        int frontRightTarget;
        int backRightTarget;

        if (opModeIsActive()) {

            driveFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
            driveFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
            driveBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
            driveBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);
            frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);
            backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);
            backRightTarget = driveBackRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);

            // set target position to each motor
            driveFrontLeft.setTargetPosition(frontLeftTarget);
            driveFrontRight.setTargetPosition(frontRightTarget);
            driveBackLeft.setTargetPosition(backLeftTarget);
            driveBackRight.setTargetPosition(backRightTarget);

            // Turn on run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            driveFrontLeft.setPower(Math.abs(speed));
            driveFrontRight.setPower(Math.abs(speed));
            driveBackLeft.setPower(Math.abs(speed));
            driveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
                telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
                telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
                telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            //Turn off run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void strafeDriveEncoder(double speed, double distance, String direction) {
        int frontLeftTarget = 0;
        int backLeftTarget = 0;
        int frontRightTarget = 0;
        int backRightTarget = 0;

        switch (direction) {
            case "LEFT":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.45);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.45);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.45);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.45);
                break;
            case "RIGHT":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.45);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.45);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.45);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.45);
                break;
        }
        if (opModeIsActive()) {

            // set target position to each motor
            driveFrontLeft.setTargetPosition(frontLeftTarget);
            driveFrontRight.setTargetPosition(frontRightTarget);
            driveBackLeft.setTargetPosition(backLeftTarget);
            driveBackRight.setTargetPosition(backRightTarget);

            // Turn on run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            driveFrontLeft.setPower(Math.abs(speed));
            driveFrontRight.setPower(Math.abs(speed));
            driveBackLeft.setPower(Math.abs(speed));
            driveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
                telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
                telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
                telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            //Turn off run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    // NOT GETTING ANY ENCODER READINGS
    public void intakeSlideEncoder(double speed, double distanceCM) {
        int target;

        if (opModeIsActive()) {

            intakeSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            target = intakeSlide.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_INTAKE);

            // set target position to each motor
            intakeSlide.setTargetPosition(target);

            // Turn on run to position
            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeSlide.setPower(Math.abs(speed));

            // Display it for the driver.
            telemetry.addData("INTAKE SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, intakeSlide.getCurrentPosition());
            telemetry.update();
            sleep(2000);

            while (opModeIsActive() &&
                    (intakeMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("INTAKE SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, intakeSlide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            intakeSlide.setPower(0);

            //Turn off run to position
            intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

}
