package org. firstinspires. ftc.       teamcode.  TeleOp;

import  com. qualcomm.      hardware.  bosch.     BNO055IMU;
import  com. qualcomm.      robotcore. eventloop. opmode.    LinearOpMode;
import  com. qualcomm.      robotcore. eventloop. opmode.    TeleOp;
import  com. qualcomm.      robotcore. hardware.  DcMotor;
import  com. qualcomm.      robotcore. hardware.  Servo;

import  org. firstinspires. ftc.       robotcore. external.   navigation.   AngleUnit;
import  org. firstinspires. ftc.       robotcore. external.   navigation.   AxesOrder;
import  org. firstinspires. ftc.       robotcore. external.   navigation.   AxesReference;
import  org. firstinspires. ftc.       robotcore. external.   navigation.   Orientation;


@TeleOp
public class manualBirdSwitch extends LinearOpMode{

    private  BNO055IMU       emu;
    private  Orientation     angles;
    private  DcMotor         FLwheel;
    private  DcMotor         FRwheel;
    private  DcMotor         BLwheel;
    private  DcMotor         BRwheel;
    private  DcMotor         lift;
    private  Servo           claw;
    //private  DigitalChannel  digitalTouch;
    //private  DistanceSensor  sensorColorRange;
    //private  Servo           servoTest;

    @Override
    public void runOpMode() {

        // this is where you initialize the motor variables and point them to the correct hardware
        FLwheel           = hardwareMap.get(DcMotor.class, "LFront" );
        FRwheel           = hardwareMap.get(DcMotor.class, "RFront" );
        BLwheel           = hardwareMap.get(DcMotor.class, "LRear"  );
        BRwheel           = hardwareMap.get(DcMotor.class, "RRear"  );
        lift              = hardwareMap.get(DcMotor.class, "Lift"   );
        claw              = hardwareMap.get(  Servo.class, "Claw"   );

        claw.setPosition(0.45F);

        sleep(500);

        //digitalTouch      = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange  = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest         = hardwareMap.get(Servo.class, "servoTest");

        // this is where you initialize the gyroscope
        BNO055IMU.Parameters parameters = new   BNO055IMU.Parameters(   );
        parameters.angleUnit            =       BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  =       "BNO055IMUCalibration.json";
        emu = hardwareMap.get(BNO055IMU.class,  "imu"         );
        emu.initialize(parameters);

        // this is where you declare all the variables
        double  fl         =  0;
        double  fr         =  0;
        double  bl         =  0;
        double  br         =  0;
        int     speed      =  5;
        double  usedSpeed  =  0.6;
        boolean aH         =  false;
        boolean bH         =  false;
        float   loki       =  0F;
        float   thor       =  0F;
        float   direction  =  0F;
        double  average    =  0D;
        double  lAverage   =  0D;
        float   cherryPie  =  (float)Math.PI;
        double  usedX      =  0D;
        double  usedY      =  0D;
        float   looki       =  0F;
        double  liftS      =  0D;
        int     lycel      =  0;
        float   turnSpeed  =  0F;
        double  clawS      =  0.25F;
        int     cole      =  0;
        float   threshold  =  (float)Math.sqrt(2) / 2;
        boolean lbh        =  true;
        float   heading;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        // this loop is where you put all the code that runs during the opMode
        while (opModeIsActive()) {

            // this updates the gyroscope every frame
            angles = emu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;

            // the following if statements can increase, decrease, or reset the speed of the robot
            speed = 5;

            if (this.gamepad1.a){

                speed = 9;

            }

            if (this.gamepad1.b){

                speed = 1;

            }

            if(this.gamepad1.y){

                emu.initialize(parameters);

            }

            usedSpeed = (speed + 1) / 10.;

            // this converts right stick game-pad xy to a direction in radians that we can compare to the gyro direction
//            if (this.gamepad1.left_stick_button && lbh){
//
//                //togCard = !togCard;
//                lbh     = false;
//
//            }
//
//            if (!this.gamepad1.left_stick_button){
//
//                lbh = true;
//
//            }

            loki      = (float) Math.atan( (-(this.gamepad1.right_stick_y))/ this.gamepad1.right_stick_x);

            if (this.gamepad1.right_stick_x >= 0){

                loki += cherryPie;

            }

            if(Double.isNaN(loki)){
                loki = 0;
            }

            looki = loki;

            while (looki > (2F * cherryPie)){

                looki -= (2F * cherryPie);

            }

            while (looki < 0){

                looki += (2F * cherryPie);

            }

















            // this converts game-pad left stick xy to a direction in radians so we can add the gyro direction
            thor      = (float) Math.atan( (-(this.gamepad1.left_stick_y)) / this.gamepad1.left_stick_x );
            thor     += cherryPie / 2;

            if (this.gamepad1.left_stick_x >= 0){

                thor += cherryPie;

            }

            // this subtracts the gyro angle to get the birds-eye offset direction
            if(Double.isNaN(thor)){
                thor = 0;
            }

            direction = thor - (cherryPie/2) - (heading / 180F * cherryPie);

            // this makes sure the direction is always from 0 to 2pi as opposed to negatives or above 2pi because that would be difficult to work with
            while (direction > (2F * cherryPie)){

                direction -= (2F * cherryPie);

            }

            while (direction < 0){

                direction += (2F * cherryPie);

            }










            //this changes the speed based on how far you moved the stick
            average  =  Math.sqrt(Math.pow(this.gamepad1.left_stick_x, 2F) + Math.pow(this.gamepad1.left_stick_y, 2F));

            // this converts the left stick directions that have been gyro-adjusted to coordinates the wheels can use
            usedY    =  -Math.cos(direction);
            usedX    =  -Math.sin(direction);

            //this calculates turn-speed based on the direction you want to point
            turnSpeed = (cherryPie / 2) + looki - (heading / 180F * cherryPie);
            
            while (turnSpeed > cherryPie){
                turnSpeed -= (2 * cherryPie);
            }

            while (turnSpeed < -cherryPie){
                turnSpeed += (2 * cherryPie);
            }

            //this changes turn-speed based on how far you moved the stick
            lAverage   =  Math.sqrt( Math.pow( this.gamepad1.right_stick_x, 2F ) + Math.pow( this.gamepad1.right_stick_y, 2F ) );

            // these move the lift
            if ( ( ( this.gamepad2.left_stick_y < -0.2 ) /* && lycel < 1800*/ ) || ( ( this.gamepad2.left_stick_y > 0.2 ) /* && lycel > 0*/ ) ) {

                liftS  = 10 * this.gamepad2.left_stick_y;
                lycel +=      this.gamepad2.left_stick_y / Math.abs( this.gamepad2.left_stick_y );

            } else {

                liftS  = 0;

            }

            if (this.gamepad2.left_trigger > 0.1 && clawS > 0.45F){

                clawS  = claw.getPosition()-(0.005F * this.gamepad2.left_trigger);
                cole--;

            } else if (this.gamepad2.right_trigger > 0.1 && clawS < 0.8F ){

                clawS  = claw.getPosition()+(0.005F * this.gamepad2.right_trigger);
                cole++;

            }else {
                
                clawS  = claw.getPosition();
                
            }

            if(Double.isNaN(average)){
                average = 0;
            }

            if(Double.isNaN(lAverage)){
                lAverage = 0;
            }



            // these set all the motor values
            fl  = ((( -usedX  +  usedY ) * average ) + ( turnSpeed * lAverage )) * usedSpeed;
            //* usedSpeed * average  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger;
            fr  = (((  usedX  +  usedY ) * average ) - ( turnSpeed * lAverage )) * usedSpeed;
            //* usedSpeed * average  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger;
            br  = ((( -usedX  +  usedY ) * average ) - ( turnSpeed * lAverage )) * usedSpeed;
            //* usedSpeed * average  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger;
            bl  = (((  usedX  +  usedY ) * average ) + ( turnSpeed * lAverage )) * usedSpeed;
            //* usedSpeed * average  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger;

            // this moves the motors
            FLwheel.setPower(       fl   );
            FRwheel.setPower(      -fr   );
            BLwheel.setPower(       bl   );
            BRwheel.setPower(      -br   );
            lift.   setPower(    liftS   );
            claw.   setPosition( clawS   );

            // this displays things to the screen
            telemetry.addData(  "Status"        , "Running"    );
            telemetry.addData(  "Speed"         , speed              );
            telemetry.addData(  "Heading"       , claw.getPosition()            );
            telemetry.update();

        }

    }

}