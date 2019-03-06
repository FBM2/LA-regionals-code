//Package statement
package org.firstinspires.ftc.teamcode.Auto.CurrentAuto;

//Import Statements
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

/** CraterAuto2 created by Isaac Dienstag for ftc team 9804 Bombsquad.
 * This is the main class for autonomous when starting on the side facing the crater. This class
 * extends TensorFlow because it uses the phone's camera along with vuforia in order to determine
 * the position of the gold block in autonomous. */

//Declaration for display on the driver station
//@Disabled
@Autonomous(name = "Crater Auto 2")
public class CraterAuto2 extends TensorFlow {

    private Recognition lowestGold;
    private boolean leftBlock = false, rightBlock = false, centerBlock = false;

    //Main OpMode method
    public void runOpMode() {

        //Create a new thread for our telemetry that will run parallel to this one
        TelemetryThread th = new TelemetryThread("TelThread");


        //Initialize all motors, servos, and sensors (including gyro imu, Tfod, and vuforia)
        //with our name as Crater Auto 2. Transition to TeleopMain after this opmode is over
        initAll("Crater Auto 2", "TeleopMain");

        waitForStart();//Wait for us to start the autonomous
        resetStartTime();//Reset the start time once we press play

        lowestGold = getGoldBlock(1);//Take 1 second to look for the lowest gold block on screen

        if(lowestGold == null || ((int)lowestGold.getTop()) > 900) { //Return the l/c/r position of the gold mineral
            setGoldMineralPosTelemetry("Right");
            rightBlock = true;
        }else if(lowestGold.getTop() < 450) {
            setGoldMineralPosTelemetry("Left");
            leftBlock = true;
        }else {
            setGoldMineralPosTelemetry("Center");
            centerBlock = true;
        }telemetry.update();

        //Start the telemetry child thread that will continuously return our current angle on screen
        //as well as set the angle to a variable for use in our imu turns
        th.startThread();

        //We start hanging, so we call the method dropFromHang(), which pulls out the lock,
        //lowers us down, and unlaches us from the lander, followed by an imu turn to make us
        //parallel to the lander
        dropFromHang();

        //Run our robot to the corner to drop our marker
        driveWithEncoders(17,.4, 5, "Drive to Gap");
        rotate(80,.35,3, "Turn to wall");
        driveWithEncoders(65,.5, 3, "Drive to Wall");
        rotate(115, .35, 5, "Turn to Depot");
        driveWithEncoders(25,.4,5, "Drive to Depot");

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the field, messing us up and damaging the intake
        dropMarker();

        //Realign to make sure we do not hit our teammate's right block and drive back to our crater
        rotate(130  ,.35,5, "Allign to wall");

        //Drive back and realign with the lander
        driveWithEncoders(37,-.4,3, "Drive away from Depot");
        pause(.1);
        rotate(95, .35, 5, "turn towards center");

        //If we didn't see a gold block, or the gold block was on the very far right of our screen
        //We assume the gold block was in the right position, and turn to the right to hit it
        if(rightBlock) {
            driveWithEncoders(70,-.5, 3, "Drive to right block");
            rotate(-5, .35, 7, "Turn towards right block");
        }
        //If the gold block was in the left porton of the screen, we assume the block is left, and turn towards it
        else if(leftBlock) {
            driveWithEncoders(40,-.5, 3, "Drive to left block");
            rotate(15, .35, 7, "Turn towards left block");
        }
        //If the block was not in the v ery far right or far left, but we still saw it
        //we assume the block is in the center, and do not turn towards either direction
        else {
            driveWithEncoders(48,-.5, 3, "Drive to center block");
            rotate(7, .35, 7, "Turn towards center block");
        }

        //Drive forward to hit the block and partially park in the crater, but if the block is center
        //Drive forward for a shorter period so we do not wedge the block on the crater
        if(!rightBlock)
            driveWithEncoders(30, .4, 2, "Drive forward and hit block");
        else
            driveWithEncoders(15,.35,2, "Drive forward and hit block");

        //Extend our extender down to ensure we are parked
        setExtenderPower(.25);
        pause(1);
        setExtenderPower(0);

    } //Ends runOpMode method
} //Ends class

