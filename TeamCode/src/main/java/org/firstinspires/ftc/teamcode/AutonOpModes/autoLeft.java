package org.firstinspires.ftc.teamcode.AutonOpModes;


import  com.    qualcomm.       hardware.   bosch.                 BNO055IMU;
import  com.    qualcomm.       robotcore.  eventloop.             opmode.             Autonomous;
import  com.    qualcomm.       robotcore.  eventloop.             opmode.             LinearOpMode;
import  com.    qualcomm.       robotcore.  hardware.              DcMotor;
import  com.    qualcomm.       robotcore.  hardware.              Servo;
//import com.   qualcomm.       robotcore.  util.                  Hardware;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     AngleUnit;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     AxesOrder;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     AxesReference;
import  org.    firstinspires.  ftc.        robotcore.             external.           navigation.     Orientation;


@Autonomous(name="autoLeft", group="Test")
public class autoLeft extends LinearOpMode {
    private         BNO055IMU       emu;
    private         Orientation     angles;
    private  static DcMotor         FLwheel;
    private  static DcMotor         FRwheel;
    private  static DcMotor         BLwheel;
    private  static DcMotor         BRwheel;
    private         DcMotor         lift;
    private         Servo           claw;

    //private  DigitalChannel  digitalTouch;
    //private  DistanceSensor  sensorColorRange;
    //private  Servo           servoTest;

    static int timing = 0;
    static int rTime  = 0;
    static float fl = 0;
    static float bl = 0;
    static float fr = 0;
    static float br = 0;
    static float[] powers = {0,0,0,0};
    static float DtT = (float)(160);
    static float CtT = (float)(64000/393);

    private static void runMotors(float fl, float bl, float fr, float br, int time){

        int timing = 0;

        while (timing <  time) {
            FLwheel.setPower(-fl);
            BLwheel.setPower(-bl);
            FRwheel.setPower( fr);
            BRwheel.setPower( br);
            timing++;
        }
        timing = 0;
        while(timing < 500){
            FLwheel.setPower( fl * 0.9);
            BLwheel.setPower( bl * 0.9);
            FRwheel.setPower(-fr * 1.1);
            BRwheel.setPower(-br * 1.1);
            timing++;
        }

        FLwheel.setPower(0);
        BLwheel.setPower(0);

        timing = 0;

        while (timing < 800){

            FRwheel.setPower(-fr * 1.1);
            BRwheel.setPower(-br * 1.1);
            timing++;

        }

        FRwheel.setPower(0);
        BRwheel.setPower(0);

        timing = 0;

        while (timing < 30000){

            timing++;
            FLwheel.setPower(0);
            BLwheel.setPower(0);
            FRwheel.setPower(0);
            BRwheel.setPower(0);

        }

        timing = 0;


    }


//
//    private static void movecm(float cm, float x, float y, float speed){
//
//        FLwheel.setTargetPosition((int)(cm * 50));
//        BLwheel.setTargetPosition((int)(cm * 50));
//        FRwheel.setTargetPosition((int)(cm * 50));
//        BRwheel.setTargetPosition((int)(cm * 50));
//
//
//
//    }


    @Override
    public void runOpMode() {

        FLwheel           = hardwareMap.get( DcMotor.class, "LFront" );
        FRwheel           = hardwareMap.get( DcMotor.class, "RFront" );
        BLwheel           = hardwareMap.get( DcMotor.class, "LRear"  );
        BRwheel           = hardwareMap.get( DcMotor.class, "RRear"  );
        lift              = hardwareMap.get( DcMotor.class, "Lift"   );
        claw              = hardwareMap.get(   Servo.class, "Claw"   );

//        FLwheel.setDirection(DcMotor.Direction.REVERSE);
//        BLwheel.setDirection(DcMotor.Direction.REVERSE);
//
//        FLwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FRwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BLwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BRwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //digitalTouch      = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange  = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest         = hardwareMap.get(Servo.class, "servoTest");

        BNO055IMU.Parameters parameters = new   BNO055IMU.Parameters();
        parameters.angleUnit            =       BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  =       "BNO055IMUCalibration.json";
        emu                             =       hardwareMap.get(BNO055IMU.class,"imu");
        emu.initialize(parameters);

        timing = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        // this loop is where you put all the code that runs during the opMode
        if (opModeIsActive()) {

            angles = emu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



            //runMotors((float)0.9,(float)0.9,1,1,(int)(50 * CtT));


            //runMotors((float)-4.5,(float)-4.5,5,5,(int)(180 * DtT));

         //   lift.setPower(  -10   );
            claw.setPosition(1);

            rTime = 0;

            while (rTime < 100000){

                claw.setPosition(1);
                rTime++;

            }

            rTime = 0;


            while (rTime < 530000){

                rTime++;
                lift.setPower(-10);

            }
            lift.setPower(0);

            runMotors((float)0.176667,(float)0.176667,(float)0.182,(float)0.182,(int)(1120 * CtT));




            //runMotors((float)0.18,(float)-0.18,-(float)0.2,(float)0.2,(int)(300 * CtT));

            //rTime = 0;

            claw.setPosition(0);

            //runMotors((float)-0.18,(float) 0.18, (float) 0.2, (float)-0.2, (int)(1000  * CtT));
            runMotors((float)-0.176667,(float)-0.176667,(float)-0.182,(float)-0.182,(int)(1090 * CtT));

            runMotors((float)-0.18,(float)-0.18, (float) 0.2, (float) 0.2, (int)(500  * DtT));

            rTime = 0;

            while (rTime < 400000){

                lift.setPower(10);
                rTime++;

            }

            lift.setPower(0);

            telemetry.addData("Status"    , "Running"     );
            telemetry.addData("Heading"   , timing           );
            telemetry.update();

        }

    }

}