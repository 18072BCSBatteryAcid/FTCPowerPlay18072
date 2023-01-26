package org.    firstinspires.    ftc.        teamcode.              TeleOp;

import  com.    qualcomm.         hardware.   bosch.                 BNO055IMU;
import  com.    qualcomm.         robotcore.  eventloop.             EventLoopManager;
import  com.    qualcomm.         robotcore.  eventloop.             opmode.             Disabled;
import  com.    qualcomm.         robotcore.  eventloop.             opmode.             LinearOpMode;
import  com.    qualcomm.         robotcore.  eventloop.             opmode.             TeleOp;
import  com.    qualcomm.         robotcore.  hardware.              DcMotor;
import  com.    qualcomm.         robotcore.  hardware.              DcMotorSimple;
import  com.    qualcomm.         robotcore.  hardware.              DigitalChannel;
import  com.    qualcomm.         robotcore.  hardware.              DistanceSensor;
import  com.    qualcomm.         robotcore.  hardware.              Gyroscope;
import  com.    qualcomm.         robotcore.  hardware.              HardwareMap;
import  com.    qualcomm.         robotcore.  hardware.              I2cWarningManager;
import  com.    qualcomm.         robotcore.  hardware.              Servo;
import  com.    qualcomm.         robotcore.  robot.                 Robot;
import  com.    qualcomm.         robotcore.  util.                  ElapsedTime;
//import com.   qualcomm.         robotcore.  util.                  Hardware;
import  com.    qualcomm.         robotcore.  util.                  Range;
import  com.    qualcomm.         robotcore.  util.                  RobotLog;
import  com.    qualcomm.         robotcore.  util.                  ThreadPool;
import  org.    checkerframework. checker.    units.                 qual.               radians;
import  org.    firstinspires.    ftc.        robotcore.             external.           navigation.     AngleUnit;
import  org.    firstinspires.    ftc.        robotcore.             external.           navigation.     AxesOrder;
import  org.    firstinspires.    ftc.        robotcore.             external.           navigation.     AxesReference;
import  org.    firstinspires.    ftc.        robotcore.             external.           navigation.     Orientation;
import  org.    firstinspires.    ftc.        robotcore.             internal.           opmode.         TelemetryInternal;
import  java.   lang.             Math;
import  java.   util.             concurrent. CancellationException;
import  java.   util.             concurrent. ExecutorService;
import  java.   util.             concurrent. TimeUnit;
import  java.   util.             Stack;

@TeleOp
public class ClawInit extends LinearOpMode {

    private Servo claw;
    
    @Override
    public void runOpMode() {

        claw = hardwareMap.get(Servo.class, "Claw");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        claw.setPosition(0.5);
        
        }

}