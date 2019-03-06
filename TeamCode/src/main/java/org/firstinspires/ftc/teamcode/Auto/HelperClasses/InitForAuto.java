//Package statement
package org.firstinspires.ftc.teamcode.Auto.HelperClasses;

//Import statements
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutoTransitioner;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/** InitForAuto created by Isaac Dienstag for ftc team 9804 Bombsquad.
 * This class contains all of the private instance variables for the baseline autonomous
 * and all of the initialization methods (methods we run on initialization of auto). This
 * serves to calibrate our gyroscope, initalize the motors, servos, and digital channels to
 * the heardwaremap, and gives access to the private instance variables through setters and
 * getters. */

public abstract class InitForAuto extends LinearOpMode {

    //Encoder variables
    final double ENCODER_CPR = 537.6, GEAR_RATIO = .8, WHEEL_DIAMETER = 4;
    double counts, inches = 0, rotations = 0;

    //IMU variables
    BNO055IMU imu; //IMU sensor for detecting the angle of the robot
    Orientation lastAngles = new Orientation(); //Angle state used for updating telemetry
    double globalAngle; //The angle of the robot at any given moment

    //Time variables
    double timeOne, timeTwo;

    //Motor, Servo, and Digital Channal declarations
    DcMotor right, left, lifter, extender, sweeper, hanger;
    Servo dumper, swapper;
    DigitalChannel spin, top, bottom, in;

    //Chosing variables
    boolean shouldWeDrop, chosen, confirmed;
    public static boolean runFast = false;

    //Telemetry Variables
    Telemetry.Item encoderCounts, imuAngle;
    Telemetry.Item goldMineralPos, objectsDetected;
    Telemetry.Item autoName, timeout;

    //Global Init Method. This Method initializes all parts of the robot declared, including the imu, motors, servos, and telemetry.
    //It also asks the user for choices about whether we want to drop or not. This allows us to run the same code during testing
    //but be able to test both hanging and non-hanging. This is preferable because hanging puts tension on the battery and if we
    //want to test consistantly at high battery we can skip the hanging portion of autonomous and only test the following movement
    public void initAll(String name, String chosenOpMode){

        initMotors("m2", "m1", "m5", "m4", "m3", "m6"); //Initialize motors to the hardwareMap
        initMotorDirections(REVERSE, FORWARD, REVERSE, REVERSE, FORWARD, REVERSE); //Initialize motor directions to the hardwareMap
        initServos("s1", "s2");  //Initialize servos to the hardwareMap
        initDigitals("d1", "d2", "d3", "d4");  //Initializedigital devices to the hardwareMap
        setZeroPow();//Set the zero power behavior of the motors
        telemetry.addLine("HardwareMap Initialized"); telemetry.update(); //Add telemetry so we know where we are

        init("i0"); //Initialize the IMU
        telemetry.addLine("IMU Initialized"); telemetry.update(); //Add telemetry so we know where we are

        while (!isStopRequested() && !imu.isGyroCalibrated()){ //While a stop to the op mode has not been requested and the imu is not calibrated
            sleep(50); //Sleep for 50 milliseconds
            idle(); //Mark process as swappable and lowers its priority
        }
        telemetry.addLine("IMU Gyro Initialized"); telemetry.update(); //Add telemetry so we know where we are

        swapper.setPosition(.05); //Initialze the swapper servo position

        resetAngle(); //Reset the angle of the imu once its calibrated so the current angle is 0 and all turns use this angle as a reference

        transition(chosenOpMode); //Enable the auto-transition to the chosen opMode

        chooseToDrop();//Choose whether or not to run the drop sequence
        //chooseToDrive(); //Choose whether or not to drive at full speed during teleop

        telemetry.clear(); //Clear the screen of all telemetry to prepare for our variable based telemetry

        telemetry.setAutoClear(false);//Set AutoClear to false in the telemetry so that we can keep our telemetry on screen as the auto runs

        autoName = telemetry.addData("Running: ", name);//Show what auto we are running at the top
        telemetry.addLine();//Add a space
        imuAngle = telemetry.addData("Heading Angle: ", 0);//Show our current Heading Angle
        encoderCounts = telemetry.addData("Encoder Counts: ", 0); //Show our current encoder counts
        timeout = telemetry.addData("Time left: ", 0); //Show the current timeout of whatever we are running
        telemetry.addLine(); //Add a space
        goldMineralPos = telemetry.addData("Position: ", "null"); //Show the position of the gold cube
        objectsDetected = telemetry.addData("Objects Detected: ", 0);//Show the number of objects detected
        telemetry.addLine();
        telemetry.addLine("Time left on movements:");
        telemetry.update();//Update this information onto the screen
    }

    //Initialize the Motors to the hardwaremap
    protected void initMotors(String left, String right, String lift, String extend, String sweep, String hang){
        this.left = hardwareMap.dcMotor.get(left);
        this.right = hardwareMap.dcMotor.get(right);
        lifter = hardwareMap.dcMotor.get(lift);
        extender = hardwareMap.dcMotor.get(extend);
        sweeper = hardwareMap.dcMotor.get(sweep);
        hanger = hardwareMap.dcMotor.get(hang);
    }

    //Initialize the motor directions
    protected void initMotorDirections(DcMotorSimple.Direction leftD, DcMotorSimple.Direction rightD, DcMotorSimple.Direction liftD,
                                    DcMotorSimple.Direction extendD, DcMotorSimple.Direction sweepD, DcMotorSimple.Direction hangD) {
        left.setDirection(leftD);
        right.setDirection(rightD);
        lifter.setDirection(liftD);
        extender.setDirection(extendD);
        sweeper.setDirection(sweepD);
        hanger.setDirection(hangD);
    }

    //Initialize the Servos to the hardwaremap
    protected void initServos(String dump, String swap){
        dumper = hardwareMap.servo.get(dump);
        swapper = hardwareMap.servo.get(swap);
    }

    //Initialize the DigitalChannels to the hardwaremap
    protected void initDigitals(String spinner, String isUp, String isDown, String isIn){
        spin = hardwareMap.get(DigitalChannel.class, spinner);
        top = hardwareMap.get(DigitalChannel.class, isUp);
        bottom = hardwareMap.get(DigitalChannel.class, isDown);
        in = hardwareMap.get(DigitalChannel.class, isIn);
    }

    //Initialize the IMU
    protected void init(String newIMU){

        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.mode                = BNO055IMU.SensorMode.IMU;
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled      = true;
        IMUparameters.loggingTag          = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Sensor configurations in the hardware map
        imu = hardwareMap.get(BNO055IMU.class, newIMU); //i0
        imu.initialize(IMUparameters); //Initialize the IMU
    }

    private void transition(String teleop){
        if(!teleop.equalsIgnoreCase(""))
            AutoTransitioner.transitionOnStop(this, teleop);
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void chooseToDrop(){
        chosen = false;
        confirmed = false;
        while(!confirmed && !isStopRequested()){
            if(chosen){
                if(shouldWeDrop){
                    telemetry.addLine("Dropping, A to confirm, B to cancel");telemetry.update();
                }
                else {
                    telemetry.addLine("Not Dropping, A to confirm, B to cancel");telemetry.update();
                }
                if(gamepad1.a){
                    confirmed = true;
                    telemetry.addLine();
                }
                else if(gamepad1.b){
                    chosen = false;
                }
            }
            else if(gamepad1.dpad_down){
                shouldWeDrop = false;
                chosen = true;
            }
            else if(gamepad1.dpad_up) {
                shouldWeDrop = true;
                chosen = true;
            }
            else
                telemetry.addLine("Drop? (Dpad-up = yes, Dpad-down = no)");telemetry.update();
        }
    }

    public void chooseToDrive(){
        chosen = false;
        confirmed = false;
        while(!confirmed){
            if(chosen){
                if(runFast){
                    telemetry.addLine("Full power. Press A to confirm, press B to cancel");telemetry.update();
                }
                else {
                    telemetry.addLine("3/4 power. Press A to confirm, press B to cancel");telemetry.update();
                }

                if(gamepad1.a){
                    confirmed = true;
                    telemetry.addLine("Confirmed!");telemetry.update();
                }
                else if(gamepad1.b){
                    chosen = false;
                    telemetry.addLine("Canceled!");telemetry.update();
                }
            }
            else if(gamepad1.dpad_down){
                runFast = false;
                chosen = true;
            }
            else if(gamepad1.dpad_up) {
                runFast = true;
                chosen = true;
            }
            else
                telemetry.addLine("Run at full power? (dpad-up = yes, dpad-down = 3/4 power)");telemetry.update();
        }
    }

    //Set the ZeroPowerBehavior of our motors to BRAKE so movement is more consistant and we don't slide
    protected void setZeroPow(){
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Set the power of both motors to the reverse of pow
    protected void setBothPower(double pow){
        right.setPower(-pow);
        left.setPower(-pow);
    }

    //Setter Statements
    protected void setRightPower(double pow){right.setPower(-pow);}
    protected void setLeftPower(double pow){left.setPower(-pow);}
    protected void setLifterPower(double pow){lifter.setPower(pow);}
    protected void setExtenderPower(double pow){extender.setPower(pow);}
    protected void setSweeperPower(double pow){sweeper.setPower(pow);}
    protected void setHangerPower(double pow){hanger.setPower(pow);}
    protected void setSwapperPosition(double pos){swapper.setPosition(pos);}
    protected void setDumperPosition(double pos){dumper.setPosition(pos);}
    protected void setEncoderCountsTelemetry(double num){encoderCounts.setValue(num);}
    protected void setIMUAngleTelemetry(double num){imuAngle.setValue(num);}
    protected void setGoldMineralPosTelemetry(String pos){goldMineralPos.setValue(pos);}
    protected void setTimeoutTelemetry(double num){timeout.setValue(num);}
    protected void setObjectsDetected(int num){objectsDetected.setValue(num);}
    protected void setAutoNameTelemetry(String name){autoName.setValue(name);}

    //Getter Statements
    protected double getRightPosition(){return right.getCurrentPosition();}
    protected double getLeftPosition(){return left.getCurrentPosition();}
    protected double getLeftPower(){return right.getPower();}
    protected double getRightPower(){return left.getPower();}
    protected double getLifterPower(){return lifter.getPower();}
    protected double getExtenderPower(){return extender.getPower();}
    protected double getSweeperPower(){return sweeper.getPower();}
    protected double getHangerPower(){return hanger.getPower();}
    protected double getSwapperPosition(){return swapper.getPosition();}
    protected double getDumperPosition(){return dumper.getPosition();}
    protected boolean getSpinnerPosition(){return spin.getState();}
    protected boolean getUpState(){return top.getState();}
    protected boolean getBottomState(){return bottom.getState();}
    protected boolean getInState(){return in.getState();}
    protected boolean isImuCalibrated(){return imu.isGyroCalibrated();}
    protected double getCounts(){return counts;}
    protected double getGlobalAngle(){return globalAngle;}
}
