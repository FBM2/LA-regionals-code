//Package statement
package org.firstinspires.ftc.teamcode.Auto.CurrentAuto;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

/** DepotAuto2 created by Isaac Dienstag for ftc team 9804 Bombsquad.
 * This is the main class for autonomous when starting facing the depot (formerly corner).
 * This class extends TensorFlow because it uses the phone's camera along with vuforia in order to
 * determine the position of the gold block in autonomous. */

//Declaration for display on the driver station
//@Disabled
@Autonomous(name = "Depot Auto 2")
public class DepotAuto2 extends TensorFlow {

    //Variable declarations
    private Recognition lowestGold;
    private boolean centerBlock = false, rightBlock = false, leftBlock = false;

    //Main OpMode method
    public void runOpMode() {

        //Create a new thread for our telemetry that will run parallel to this one
        TelemetryThread th = new TelemetryThread("TelThread");

        //Initialize all motors, servos, and sensors (including gyro imu, Tfod, and vuforia)
        //with our name as Depot Auto 2. Transition to TeleopMain after this opmode is over
        initAll("Depot Auto 2", "");

        //Start the telemetry child thread that will continuously return our current angle on screen
        //as well as set the angle to a variable for use in our imu turns
        th.startThread();

        waitForStart();//Wait for us to start the autonomous
        resetStartTime();//Reset the start time once we press play

        lowestGold = getGoldBlock(1);//Take 1 second to look for the lowest gold block on screen

        //If we didn't see a gold block, or the gold block is on the very far left of our screen
        //We assume the gold block was in the left position
        if(lowestGold == null || ((int)lowestGold.getTop()) < 100) {
            setGoldMineralPosTelemetry("Left");
            leftBlock = true;
        }//If the gold block was in the right portion of the screen, we assume the block is right
        else if(lowestGold.getTop() > 500) {
            setGoldMineralPosTelemetry("Right");
            rightBlock = true;
        }//Else, that means the block is not to the right or to the left, so it must be center
        else {
            setGoldMineralPosTelemetry("Center");
            centerBlock = true;
        }telemetry.update();//update the telemetry with our new block position

        //We start hanging, so we call the method dropFromHang(), which pulls out the lock,
        //lowers us down, and unlaches us from the lander, followed by an imu turn to make us
        //parallel to the lander
        dropFromHang();

        //If the block is on the right we turn to the right, drive forward and hit the block.
        //We then back up turn clockwise so our intake faces our own team's crater and drive
        //backwards until we hit the wall. We then turn towards counter-clockwise until our intake
        //faces our depot and drive forwards towards it. We move like this in order to abide by
        //safepaths and ensure we do not hit our alliance partner.
        if(rightBlock) {
            rotate(-30, .35, 7, "Turn towards right block");
            driveWithEncoders(38, .4, 3, "Drive and hit right block");
            driveWithEncoders(13,-.4,3, "Back up");
            rotate(-83, .35, 5, "Turn towards wall");
            driveWithEncoders(60,-.4,5, "Drive towards wall");
            rotate(-40,.34,5, "Turn towards depot");
            driveWithEncoders(40,.4,5, "Drive towards depot:");
        }//If the block is on the left we turn to the left, drive forward and hit the block.
        //We then immediately turn towards our depot and drive forward towards it.
        else if(leftBlock) {
            rotate(20, .35, 7, "Turn towards left block");
            driveWithEncoders(40,.4,3, "Drive and hit block");
            rotate(-15,.35,3, "Turn towards depot");
            driveWithEncoders(20,.4,2, "Drive towards depot");
        }//If the block is in the center we do not turn towards either direction; instead we just
        //drive straight forward, hitting the block and lining us up with the depot
        else {
            driveWithEncoders(45, .4, 3, "Drive and hit center block");
        }

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the field, messing us up and damaging the intake
        dropMarker();

        //If the block is in the center, we back up, turn towards  the opponents crater, and drive towards it
        if(centerBlock){
            driveWithEncoders(25, -.4, 3,"Drive backwards");
            rotate(-66, .35, 5, "Turn towards wall");
            driveWithEncoders(68, -.4, 3, "Drive towards wall");
        }
        else if(rightBlock) {
            //Turn towards the opponents crater
            rotate(-40, .35, 3, "Turn towards crater");
            //Drive backwards towards the opposing team's crater to park
            driveWithEncoders(75, -.5, 4, "Drive towards crater");
        }
        else {
            //Turn towards the opponents crater
            rotate(-45, .35, 3, "Turn towards crater");
            //Drive backwards towards the opposing team's crater to park
            driveWithEncoders(75, -.5, 4, "Drive towards crater");
        }
    } //Ends runOpMode method
} //Ends class
