package org.firstinspires.ftc.teamcode.OpenCV;

import  android.annotation.SuppressLint;
import  android.os.Build;

import  com.qualcomm.hardware.bosch.BNO055IMU;
import  com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.hardware.DcMotor;
import  com.qualcomm.robotcore.hardware.Servo;
import  com.qualcomm.robotcore.util.ElapsedTime;
import  com.qualcomm.robotcore.util.Range;

import  org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import  org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import  org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import  org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import  org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import  org.openftc.apriltag.AprilTagDetection;
import  org.openftc.easyopencv.OpenCvCamera;
import  org.openftc.easyopencv.OpenCvCameraFactory;
import  org.openftc.easyopencv.OpenCvCameraRotation;

import  java.util.ArrayList;
import  java.util.stream.IntStream;

@Autonomous(name="Just work this time", group="ATAuto")
public class encoderTestWAprilTags extends LinearOpMode {

    static double fLPowerAdjustment = 1;
    static double fRPowerAdjustment = 1;
    static double rLPowerAdjustment = 1;
    static double rRPowerAdjustment = 1;

    private DcMotor fLWheel;
    private DcMotor fRWheel;
    private DcMotor bLWheel;
    private DcMotor bRWheel;
    private DcMotor lift;
    private Servo claw;
    private BNO055IMU imu;

    private double robotHeading = 0;
    private double headingOffset = -90;
    private double headingError = 0;

    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double fLSpeed = 0;
    private double fRSpeed = 0;
    private double bLSpeed = 0;
    private double bRSpeed = 0;
    private int fLTarget = 0;
    private int fRTarget = 0;
    private int bLTarget = 0;
    private int bRTarget = 0;

    static final double COUNTS_PER_MOTOR_REV = 537.7;// eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159265358979323);

    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.2;
    static final double HEADING_THRESHOLD = 1.0;

    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    int zone;
    String zoneName;

    final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.05;

    final int LEFT_ZONE = 4;
    final int MIDDLE_ZONE = 8;
    final int RIGHT_ZONE = 12;

    AprilTagDetection tagOfInterest = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            fLTarget = fLWheel.getCurrentPosition() + moveCounts;
            fRTarget = fRWheel.getCurrentPosition() + moveCounts;
            bLTarget = bLWheel.getCurrentPosition() + moveCounts;
            bRTarget = bRWheel.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            fLWheel.setTargetPosition(fLTarget);
            fRWheel.setTargetPosition(fRTarget);
            bLWheel.setTargetPosition(bLTarget);
            bRWheel.setTargetPosition(bRTarget);

            fLWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fRWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (fLWheel.isBusy() && fRWheel.isBusy() && bLWheel.isBusy() && bRWheel.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            fLWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bLWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        fLSpeed = drive + (turn * fLPowerAdjustment);
        fRSpeed = drive + (turn * fRPowerAdjustment);
        bLSpeed = drive - (turn * rLPowerAdjustment);
        bRSpeed = drive - (turn * rRPowerAdjustment);

        // Scale speeds down if either one exceeds +/- 1.0;
        //                        This code has a six pack of abs.
        double maxF = Math.max(Math.abs(fLSpeed), Math.abs(fRSpeed));
        double maxB = Math.max(Math.abs(bLSpeed), Math.abs(bRSpeed));
        double max = Math.max(Math.abs(maxF), Math.abs(maxB));
        if (max > 1.0) {
            fLSpeed /= max;
            fRSpeed /= max;
            bLSpeed /= max;
            bRSpeed /= max;
        }

        fLWheel.setPower(fLSpeed);
        fRWheel.setPower(fRSpeed);
        bLWheel.setPower(bLSpeed);
        bRWheel.setPower(bRSpeed);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);

    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }



    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

        }

        // Stop all motion;
        moveRobot(0, 0);
    }



    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    @Override
    public void runOpMode() {

        fLWheel = hardwareMap.get(DcMotor.class,"LFront");
        fRWheel = hardwareMap.get(DcMotor.class,"RFront");
        bLWheel = hardwareMap.get(DcMotor.class,"LRear");
        bRWheel = hardwareMap.get(DcMotor.class,"RRear");
        lift    = hardwareMap.get(DcMotor.class,"Lift");
        claw    = hardwareMap.get(Servo.class,"Claw");

        fLWheel.setDirection(DcMotor.Direction.REVERSE);
        fRWheel.setDirection(DcMotor.Direction.REVERSE);
        bLWheel.setDirection(DcMotor.Direction.FORWARD);
        bRWheel.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(  );
        parameters.angleUnit = BNO055IMU. AngleUnit. DEGREES;
        imu = hardwareMap. get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        fLWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT_ZONE || tag.id == MIDDLE_ZONE || tag.id == RIGHT_ZONE) {
                    tagOfInterest = tag;
                    tagFound      = true;
                    break;
                }
            }
            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

        fLWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        //Update the telemetry and the stored zone
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            zone = tagOfInterest.id;

            if(zone == LEFT_ZONE) {
                zoneName = "Left";
            }else if(zone == MIDDLE_ZONE) {
                zoneName = "Middle";
            }else if(zone == RIGHT_ZONE) {
                zoneName = "Right";
            }

            telemetry.addData(">Zone: ", zoneName);

        }else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
            //Sets the middle zone as the default if no tag is detected
            zone = MIDDLE_ZONE;
        }

        if(opModeIsActive()) {

            claw.setPosition(0.45F);

            sleep(500);

            turnToHeading(TURN_SPEED, 90);

            sleep(2000);

            //park in the correct zone by moving one tile to the left or right
            /*
            switch (zone){
                case LEFT_ZONE:
                    turnToHeading(TURN_SPEED, 90.0);
                    holdHeading(TURN_SPEED, 0.0, 0.5);
                    driveStraight(DRIVE_SPEED, 24, 0.0);
                    holdHeading(TURN_SPEED, 0.0, 0.5);
                    turnToHeading(TURN_SPEED, 0.0);
                    break;
                case RIGHT_ZONE:
                    turnToHeading( TURN_SPEED, 90.0);
                    holdHeading( TURN_SPEED, 0.0, 0.5);
                    driveStraight( DRIVE_SPEED, -24, 0.0);
                    holdHeading( TURN_SPEED, 0.0, 0.5);
                    turnToHeading( TURN_SPEED, 0.0);
                    break;
            }*/
/*
            driveStraight(DRIVE_SPEED, 24, 0.0);
             holdHeading(TURN_SPEED, 0.0, 0.5);
            turnToHeading(TURN_SPEED, 90.0);
            holdHeading(TURN_SPEED, 0.0, 0.5);*/
        }


    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine( String.format("\nDetected tag ID=%d", detection. id));
        telemetry.addLine( String.format("Translation X: %.2f feet", detection. pose.x*FEET_PER_METER));
        telemetry.addLine( String.format("Translation Y: %.2f feet", detection. pose.y*FEET_PER_METER));
        telemetry.addLine( String.format("Translation Z: %.2f feet", detection. pose.z*FEET_PER_METER));
        telemetry.addLine( String.format("Rotation Yaw: %.2f degrees", Math.toDegrees( detection.pose.yaw)));
        telemetry.addLine( String.format("Rotation Pitch: %.2f degrees", Math.toDegrees( detection.pose.pitch)));
        telemetry.addLine( String.format("Rotation Roll: %.2f degrees", Math.toDegrees( detection.pose.roll)));
    }
}