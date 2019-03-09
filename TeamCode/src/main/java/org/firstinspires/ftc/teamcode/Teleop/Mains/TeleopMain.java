//Package statements
package org.firstinspires.ftc.teamcode.Teleop.Mains;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Teleop.Objects.DriveTrain;
import org.firstinspires.ftc.teamcode.Teleop.Objects.Grabber;
import org.firstinspires.ftc.teamcode.Teleop.Objects.Hanger;
import org.firstinspires.ftc.teamcode.Teleop.Objects.Lifter;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/** TeleopMain created by Isaac Dienstag for Team 9804 Bombsquad
 * This class contains all of the functions for teleop, including use of the Drivetrain, grabbers,
 * Lifters, and hanger. It also returns telemetry continuously for all of the motor powers, servo
 * positions, and current states of digital sensors. It uses objects to control these functions
 * and minimizes the use of its own variables and and tries to simplify code whenever possible. */

//Declaration for display on the driver station
@TeleOp(name = "TeleopMain")

public class TeleopMain extends OpMode {


    private DriveTrain DT; private Grabber GB; private Lifter LT; private Hanger HG;//Robot object declarations
    private int loopCounter; private double timeOne, timeTwo;//Telemetry variables
    private boolean currentStatus = false, previousStatus = false, inDrop = false, runningDrop = false, runningUp = false, ran = false; //toggle variables
    private double timeThree, timeFour;


    //Init method required by OpMode. Initializes all of our RobotObjects and initializes the servo positions where needed
    public void init(){
        //Call the constructors for our Objects with the correct hardwareMap positions and directions
        DT = new DriveTrain(hardwareMap.dcMotor.get("m2"), hardwareMap.dcMotor.get("m1"), REVERSE, FORWARD);
        GB = new Grabber(hardwareMap.dcMotor.get("m3"), hardwareMap.dcMotor.get("m4"), FORWARD, REVERSE,
                hardwareMap.digitalChannel.get("d1"), hardwareMap.digitalChannel.get("d5"), hardwareMap.crservo.get("s3"));
        LT = new Lifter(hardwareMap.dcMotor.get("m5"), REVERSE, hardwareMap.get(DigitalChannel.class, "d3"),
                hardwareMap.get(DigitalChannel.class, "d2"), hardwareMap.servo.get("s1"));
        HG = new Hanger(hardwareMap.dcMotor.get("m6"), FORWARD, hardwareMap.servo.get("s2"));

        //Initialize servo positions and motor zero power behaviors
        LT.dump(false, true, false, false, false);//Set the dumper servo to stay down
        DT.setZeroPow();//Set the zero power behavior of the DriveTrain motors to BRAKE so we have more precise control
        GB.setZeroPow();//Set the zero power behavior of the sweeper to BRAKE so we have more precise control
    }

    //Set timeOne to this.getRuntime() only when we press start
    public void start() {timeOne = this.getRuntime(); LT.setTime1(this.getRuntime()); loopCounter = 0;}

    //Loop method required by OpMode. This is what runs for the duration of the teleop period
    public void loop(){

        //Set gamepad2.back to a toggle so it doesn't change t/f every loop
        if(gamepad2.right_stick_button)
            currentStatus = !previousStatus;
        else
            previousStatus = currentStatus;

        //Increment the loopCounter and update timeTwo continuously
        loopCounter++;
        timeTwo = this.getRuntime();

        //Set DriveTrain powers to left stick and right stick y of gamepad1
        DT.driveFull(gamepad1.left_stick_y,gamepad1.right_stick_y);

        //Set reach control to left and right triggers of gamepad 1
        //Also set the disrupter toggle control to the b button of gamepad2
        GB.reach(gamepad1.left_trigger, gamepad1.right_trigger);
        //Set intake controls to the Y axis of the left stick on gamepad2
        GB.intake(gamepad2.left_stick_y);
        //Set the direct control of disrupter control to the a button on gamepad2
        GB.disrupt(gamepad2.y);

        //Set hanging motor power control to the y axis of the gunner's right stick
        HG.hangAndDrop(gamepad2.right_stick_y);
        //Set the up and down positions of the hanging lock mechanism to up and down on the gunner's dpad respectively
        HG.swap(gamepad2.dpad_left, gamepad2.dpad_right);

        if(currentStatus && !ran){
            if(!runningUp && !runningDrop) {
                timeThree = this.getRuntime();
                timeFour = this.getRuntime();
                if(inDrop)
                    runningUp = true;
                else
                    runningDrop = true;
                ran = true;
            }
        }
        else if(!currentStatus && ran){
            if(!runningUp && !runningDrop) {
                timeThree = this.getRuntime();
                timeFour = this.getRuntime();
                if(inDrop)
                    runningUp = true;
                else
                    runningDrop = true;
                ran = false;
            }
        }

        //At the very beginning of teleop (in the first half second), we want to run the lifter up and move the dumper servo
        //to the neutral position in order to get the dumper into the neutral position from the init position
        if(timeTwo - timeOne < .5) {
            //If we are in the first .5 seconds of teleop
            LT.lift(.5, 0, false, false); //Set the lift power of the lifter to .5 to move the lifter up
            LT.dump(false, timeTwo-timeOne < .3, false, false, false);
        }
        else if(timeTwo - timeOne < 1) {//Else if we are in the first second of teleop
            //Set the power of the lifter to -.3 to move the lifter back down to the bottom
            LT.lift(0, .3, false, false);
            LT.dump(gamepad2.x, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.b);
        }
        else if(runningDrop){ //If we are runningDrop
            timeFour = this.getRuntime(); //Continue to update the variable timeFour
            if(timeFour - timeThree < .4){ //If we have been running for less than .4 seconds
                LT.lift(.6, 0, false, false); //Lift the lifter
                LT.dump(false, true, false, false, false); //Set the dumper to down
            }
            else if(timeFour - timeThree < 1){ //If we have been running for less than 1 second
                LT.lift(0, .3, false, false); //drop the lifter
                LT.dump(false, false, false, false, true); //set the down toggle of the dumper to true
            }
            else { //else, we are not in the first second
                runningDrop = false; //exit the loop by setting runningDrop to false
                inDrop = true; //set inDrop to true to signify we are in drop
            }
        }
        else if(runningUp){ //If we are runningUp
            timeFour = this.getRuntime(); //Continue to update the variable timeFour
            if(timeFour - timeThree < .5) { //If we have been running for less than .5 seconds
                LT.lift(.5, 0, false, false); //Set the lift power of the lifter to .5 to move the lifter up
                LT.dump(false, timeTwo-timeOne < .3, false, false, false);//Set the dump position
                //to down in the first .3 seconds
            }
            else if(timeFour - timeThree < 1) { // Else if we have been running for less than 1 seconds
                //Set the power of the lifter to -.3 to move the lifter back down to the bottom
                LT.lift(0, .5, false, false);
                //Set the dumper position to the gamepad values
                LT.dump(false, false, false, false, true);
            }
            else { //Else, we are not in the first second
                runningUp = false; //Exit the loop by setting runningUp to false
                inDrop = false; //Set inDrop to false to signify we are no longer inDrop
            }
        }
        else { //Else, we are not in the first second of teleop or trying to runUp or runDown
            //Set the lifting power to the triggers and buttons of gamepad2
            LT.lift2(gamepad2.left_trigger, gamepad2.right_trigger, gamepad2.left_bumper, gamepad2.right_bumper);
            //Set the dumper positions to the buttons of gamepad2
            LT.dump(gamepad2.x, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.b);
        }

        //Telemetry for testing
        telemetry.addData("LPS:", loopCounter/(timeTwo-timeOne)); //Loops per second value
        telemetry.addLine(); //Gap between time telemetry and motor telemetry
        telemetry.addLine("Motors: "); //Motor header
        telemetry.addData("Left Power: ", DT.getLeftPower()); //The power of the left motor
        telemetry.addData("Right Power: ", DT.getRightPower()); //The power of the right motor
        telemetry.addData("Extender Power: ", GB.getExtenderPower()); //The power of the extender
        telemetry.addData("Sweeper Power: ", GB.getSweeperPower()); //The power of the sweeper
        telemetry.addData("Lift Power: ", LT.getLiftPower()); //The power of the lifter
        telemetry.addData("Hanger Power: ", HG.getHangPower()); //The power of the hanging motor
        telemetry.addLine(); //Gap between time Motor and servo telemetry
        telemetry.addLine("Servos: "); //Servo header
        telemetry.addData("Disrupter Power: ", GB.getDisrupterPower()); //The power of the disrupter CR servo
        telemetry.addData("Dump Position:", LT.getDumpPosition()); //The position of the dump servo
        telemetry.addData("Swapper Position:", HG.getSwapperPosition()); //The position of the swapper servo
        telemetry.addLine(); //Gap between servo telemetry and sensor telemetry
        telemetry.addLine("Sensors: "); //Sensor header
        telemetry.addData("Slow State: ", GB.getSlowState()); //The current state of the slow sensor
        telemetry.addData("Top State: ", LT.getTopState()); //The current state of the  atTop sensor
        telemetry.addData("Bottom State: ", LT.getBottomState()); //The current state of the atBottom sensor
        telemetry.update(); //Update every loop

    } //Ends loop
} //Ends class