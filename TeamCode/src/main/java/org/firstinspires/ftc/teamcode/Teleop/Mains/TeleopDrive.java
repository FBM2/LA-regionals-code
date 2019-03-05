//Package statements
package org.firstinspires.ftc.teamcode.Teleop.Mains;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.Objects.DriveTrain;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/** TeleopDrive created by Isaac Dienstag for Team 9804 Bombsquad
 * This class contains only the driving function for teleop. It also returns telemetry continuously
 * for the two motor powers.
 */

//Declaration for display on the driver station
@TeleOp(name = "Teleop Drive")
public class TeleopDrive extends OpMode {

    //Object Declarations
    private DriveTrain DT;


    //Telemetry variables
    private int loopCounter;
    private double timeOne, timeTwo;

    public void init(){
        //Initialize our objects to the hardwareMap
        DT = new DriveTrain(hardwareMap.dcMotor.get("m2"), hardwareMap.dcMotor.get("m1"), REVERSE, FORWARD);
    }

    public void start() {
        timeOne = this.getRuntime();
    }

    public void loop(){
        loopCounter++;

        //Set DriveTrain powers to left stick and right stick y
        DT.driveFull(gamepad1.left_stick_y, gamepad1.right_stick_y);

        //Telemetry for testing
        timeTwo = this.getRuntime();
        telemetry.addLine("Telemetry:");
        telemetry.addData("LPS", loopCounter/(timeTwo-timeOne));
        telemetry.addData("Left Power", DT.getLeftPower());
        telemetry.addData("Right Power", DT.getRightPower());
        telemetry.update();
    } //Ends loop
} //Ends class