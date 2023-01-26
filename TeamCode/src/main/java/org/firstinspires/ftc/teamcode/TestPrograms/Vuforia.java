package org.firstinspires.ftc.teamcode.TestPrograms;

import  com.    qualcomm.       ftcrobotcontroller.                R;
import  com.    qualcomm.       hardware.   bosch.                 BNO055IMU;
import  com.    qualcomm.       robotcore.  eventloop.             EventLoopManager;
import  com.    qualcomm.       robotcore.  eventloop.opmode.Autonomous;
import  com.    qualcomm.       robotcore.  eventloop.             opmode.             Disabled;
import  com.    qualcomm.       robotcore.  eventloop.             opmode.             LinearOpMode;
import  com.    qualcomm.       robotcore.  eventloop.             opmode.             TeleOp;
import  com.    qualcomm.       robotcore.  hardware.              DcMotor;
import  com.    qualcomm.       robotcore.  hardware.              DcMotorSimple;
import  com.    qualcomm.       robotcore.  hardware.              DigitalChannel;
import  com.    qualcomm.       robotcore.  hardware.              DistanceSensor;
import  com.    qualcomm.       robotcore.  hardware.              Gyroscope;
import  com.    qualcomm.       robotcore.  hardware.              HardwareMap;
import  com.    qualcomm.       robotcore.  hardware.              I2cWarningManager;
import  com.    qualcomm.       robotcore.  hardware.              Servo;
import  com.    qualcomm.       robotcore.  robot.                 Robot;
import  com.    qualcomm.       robotcore.  util.                  ElapsedTime;
//import com.   qualcomm.       robotcore.  util.                  Hardware;
import  com.    qualcomm.       robotcore.  util.                  Range;
import  com.    qualcomm.       robotcore.  util.                  RobotLog;
import  com.    qualcomm.       robotcore.  util.                  ThreadPool;

import org.checkerframework.checker.units.qual.radians;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     AngleUnit;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     AxesOrder;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     AxesReference;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import  org.    firstinspires.  ftc.        robotcore.             internal.           opmode.         TelemetryInternal;
import  java.   lang.           Math;
import  java.   util.           concurrent. CancellationException;
import  java.   util.           concurrent. ExecutorService;
import  java.   util.           concurrent. TimeUnit;
import  java.   util.           Stack;
@Disabled
@Autonomous(name = "Vuforia")
public class Vuforia extends LinearOpMode{

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AW7eiXv/////AAAAGYCYE5bC1E3Jo5HkEDhDbmkn4jmEULG/kFLsNYexgLBwrGrNdt0jMuWOqr5JJVMvqcETbJYl7tyG/5qfW1lzX927QU8pw8naURgogJDuCwwURKi24v7KZhx/eCrruTTLW9iPiEcEqDR/HT06boFT3GG++1bPQZxUXnEG7ypGjdNR71+IYywnL8NVTcUnGUVbK7CcFn5QflchndBXUYKX1oAoHP8J5aRjqd5r53rehVmFgz23BFioIzZBNdmsZ1M9kfJv7tPmCYGjX6WnxdIIduDjYYrovy/FAt0P+zWNV1Nn8vP0ind+criqQmnD+pfLqHNTVtFh5p/eAS/BJS2PipHWwgVEgFh9z0oS2sekoNB8";

    public void runOpMode() throws InterruptedException{

        setupVuforia();

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        visionTargets.activate();

        while(opModeIsActive()){

            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if(latestLocation != null){

                lastKnownLocation = latestLocation;

            }

            telemetry.addData("Tracking" + target.getName(),listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            telemetry.addData("Status","Running");
            telemetry.update();

        }

    }

    public void setupVuforia(){

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        target = visionTargets.get(0);
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0,0,0,0,0,0));

        phoneLocation = createMatrix(0,0,0,0,0,0);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

    }

    public OpenGLMatrix createMatrix(float u, float v, float w, float x, float y, float z){

        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES, u, v, w));

    }

    public String formatMatrix(OpenGLMatrix matrix){

        return matrix.formatAsTransform();

    }

}
