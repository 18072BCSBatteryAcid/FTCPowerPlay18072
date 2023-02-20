//TODO
//  - Try to get auto cones
//  - Figure out constant values

package org.firstinspires.ftc.teamcode.OpenCV;


import static java.lang.Math.pow;

import android.annotation.SuppressLint;
import android.os.Build;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.stream.IntStream;

@Autonomous(name="April Tag Auto", group="ATAuto")
public class aprilTagAutoBase extends LinearOpMode{

    enum LiftLevel {
        GROUND(0),
        ONE(0),
        TWO(50),
        THREE(100),
        FOUR(150),
        FIVE(200),
        GROUND_J(50),
        LOW_J(1500),
        MEDIUM_J(3000),
        HIGH_J(4600),
        CLEAR(1500 - sleepToLow);

        private final long sleepTime;

        LiftLevel(long sleepTime) {this.sleepTime = sleepTime;}

        public long getSleepTime() {return sleepTime;}
    }

    //All directions are assuming the claw is the front of the robot.
    //Motor names are given assuming the camera is the front of the robot.
    enum Direction {
        FRONT(-fBFrontLeftPowerAdjustment, -fBFrontRightPowerAdjustment,fBRearLeftPowerAdjustment, fBRearRightPowerAdjustment),
        BACK(fBFrontLeftPowerAdjustment, fBFrontRightPowerAdjustment, -fBRearLeftPowerAdjustment, -fBRearRightPowerAdjustment),
        LEFT(-lRFrontLeftPowerAdjustment,lRFrontRightPowerAdjustment, -lRRearLeftPowerAdjustment, lRRearRightPowerAdjustment),
        RIGHT(lRFrontLeftPowerAdjustment, -lRFrontRightPowerAdjustment, lRRearLeftPowerAdjustment, -lRRearRightPowerAdjustment),
        C_WISE(-turnFrontLeftPowerAdjustment, -turnFrontRightPowerAdjustment, -turnRearLeftPowerAdjustment, -turnRearRightPowerAdjustment),
        CC_WISE(turnFrontLeftPowerAdjustment, turnFrontRightPowerAdjustment, turnRearLeftPowerAdjustment, turnRearRightPowerAdjustment);

        private final double[] motorPowers;
        Direction(double... motorPowers) {this.motorPowers = motorPowers;}

        public double[] getMotorPowers() {return motorPowers;}
    }

    private Orientation angles;
    private DcMotor fLWheel;
    private DcMotor fRWheel;
    private DcMotor bLWheel;
    private DcMotor bRWheel;
    private DcMotor lift;
    private Servo claw;

    long mSPTile;
    long mSPTileLR = 245;
    long mSPTileFB = 557;
    double mSPDeg = (440/90);
    long smallMove = 150;
    double tileScale = 0.95;

    long liftSleepTime;
    boolean groundReset = false;
    LiftLevel currentLevel = LiftLevel.GROUND;
    static long sleepToLow;

    static double turnFrontLeftPowerAdjustment = 1;
    static double turnFrontRightPowerAdjustment = 1;
    static double turnRearLeftPowerAdjustment = 1;
    static double turnRearRightPowerAdjustment = 0.925;
    static double fBFrontLeftPowerAdjustment = 1;
    static double fBFrontRightPowerAdjustment = 1;
    static double fBRearLeftPowerAdjustment = 1;
    static double fBRearRightPowerAdjustment = 0.925;
    static double lRFrontLeftPowerAdjustment = 1;
    static double lRFrontRightPowerAdjustment = 0.75;
    static double lRRearLeftPowerAdjustment = 1;
    static double lRRearRightPowerAdjustment = 0.75;

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

    private double[] getMotorPowersForDirection(Direction direction) {
        return direction.getMotorPowers();
    }

    private void applyPowerToMotors(double[] motorPowers) {
        DcMotor[] motors = {fLWheel, fRWheel, bLWheel, bRWheel};
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            IntStream.range(0,motors.length).forEach(i->motors[i].setPower(motorPowers[i]));
        }
    }

    private void momentumCancel(double[] motorPowers) {
        double[] stopPowers = new double[motorPowers.length];
        for(int i = 0; i < motorPowers.length; i++){
            stopPowers[i] = -motorPowers[i];
        }
        applyPowerToMotors(stopPowers);
        sleep(40);
        stopMotors();
    }

    private void stopMotors() {
        DcMotor[] motors = {fLWheel, fRWheel, bLWheel, bRWheel};
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    //Takes a parameter value from the Direction enum
    //Moves any amount of tiles, partial tiles should work
    private void moveTiles(Direction direction, double tiles) {
        switch(direction){
            case BACK:
            case FRONT:
                mSPTile = mSPTileFB;
                break;
            case LEFT:
            case RIGHT:
                mSPTile = mSPTileLR;
                break;
        }
        double[] motorPowers = getMotorPowersForDirection(direction);
        applyPowerToMotors(motorPowers);
        sleep((long)(tiles * mSPTile * pow(tileScale, tiles - 1)));
        momentumCancel(motorPowers);
    }

    //Takes a parameter value from the Direction enum
    //C_WISE and CC_WISE should work for slight adjustments
    //Moves the robot a small amount, only used for cones or navigating around the junctions
    private void slightMove(Direction direction){
        double[] motorPowers = getMotorPowersForDirection(direction);
        applyPowerToMotors(motorPowers);
        sleep(smallMove);
        stopMotors();
    }

    //Rotates the specified number of degrees
    private void rotate(double degrees, Direction direction){
        double[] motorPowers = getMotorPowersForDirection(direction);
        applyPowerToMotors(motorPowers);
        sleep((long)(degrees * mSPDeg));
        momentumCancel(motorPowers);
    }

    //Takes a parameter from the LiftLevel enum to determine the travel distance of the lift
    /* KEY - edit if new values are added
    GROUND - lower the lift all the way
    ONE - single cone
    TWO - two stack
    THREE - three stack
    FOUR - four stack
    FIVE - five stack
    GROUND_J - ground junction
    LOW_J - low junction
    MEDIUM_J - medium junction
    HIGH_J - high junction
    CLEAR - clear cone stack/ low junction height
    */
    private void moveLift(LiftLevel targetLevel) {
        if (currentLevel == LiftLevel.GROUND || targetLevel == LiftLevel.CLEAR || targetLevel == LiftLevel.GROUND) {
            double power;
            switch (targetLevel) {
                case GROUND:
                    power = 1;
                    targetLevel = currentLevel;
                    groundReset = true;
                    liftSleepTime = (long)(targetLevel.sleepTime * 0.9);
                    break;
                default:
                    power = -1;
                    liftSleepTime = targetLevel.sleepTime;
                    groundReset = false;
                    break;
            }
            lift.setPower(power);
            sleep(liftSleepTime);
            lift.setPower(0);
            currentLevel = targetLevel;
            if(groundReset){
                currentLevel = LiftLevel.GROUND;
            }
        }
    }

    //Starting from a position centered on the targeted pole, the robot will drop a cone on the pole designated by the poleHeight parameter
    //KEY - see line 185
    private void scorePole(LiftLevel poleHeight){
        claw.setPosition(0.73F);
        moveLift(poleHeight);
        sleep(100);
        slightMove(Direction.FRONT);
        sleep(100);
        claw.setPosition(0.45F);
        sleep(500);
        slightMove(Direction.BACK);
        claw.setPosition(0.73F);
        sleep(250);
        moveLift(LiftLevel.GROUND);
    }

    @Override
    public void runOpMode(){

        fLWheel = hardwareMap.get(DcMotor.class,"LFront");
        fRWheel = hardwareMap.get(DcMotor.class,"RFront");
        bLWheel = hardwareMap.get(DcMotor.class,"LRear");
        bRWheel = hardwareMap.get(DcMotor.class,"RRear");
        lift = hardwareMap.get(DcMotor.class,"Lift");
        claw = hardwareMap.get(Servo.class,"Claw");

        claw.setPosition(0.45F);

        sleep(500);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        BNO055IMU emu = hardwareMap.get(BNO055IMU.class, "imu");
        emu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){

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
                    tagFound = true;
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        //Update the telemetry and the stored zone
        if(tagOfInterest != null){
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            zone = tagOfInterest.id;

            if(zone == LEFT_ZONE){
                zoneName = "Left";
            }else if(zone == MIDDLE_ZONE){
                zoneName = "Middle";
            }else if(zone == RIGHT_ZONE){
                zoneName = "Right";
            }

            telemetry.addData(">Zone: ", zoneName);

        }else{
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
            //Sets the middle zone as the default if no tag is detected
            zone = MIDDLE_ZONE;
        }

        if(opModeIsActive()){

            //park in the correct zone by moving one tile to the left or right

            switch (zone){
                case LEFT_ZONE:
                    moveTiles(Direction.BACK, 1);
                    break;
                case RIGHT_ZONE:
                    moveTiles(Direction.FRONT, 1);
                    break;
            }

            sleep(500);
            moveTiles(Direction.LEFT, 2.25);
            sleep(1000);
            rotate(105, Direction.CC_WISE);

        }

    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}