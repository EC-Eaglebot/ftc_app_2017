package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="forward and back test", group="Competition")
public class forward_back_test extends LinearOpMode {

    // Declare OpMode members.
    HardwareClawbot     robot   = new HardwareClawbot();
    private ElapsedTime     runtime = new ElapsedTime();
    private boolean MoveOn = false;
    private int stop = 500;


//this stuff gets the light sensor ready, taken from the sample concept_SensorColor opMode
    //ColorSensor color_sensor;
    //float[] hsvValues = new float[3];
    //final float values[] = hsvValues;

    // Get a reference to our sensor object.
    //color_sensor = hardwareMap.ColorSensor.get("color_sensor");
    
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.frontleftDrive.setPower(1);
        //robot.forward(1, 1, runtime);
      //  robot.backward(1,1,runtime);
       // double RED_FLOOR_THRESHOLD = 300;
        // double RED_CEILING_THRESHOLD = 60;

    }

}
// TOOLBOX OF FUNCTIONS
// *********************
/*
    Totally end the robot's function:       robot.stop();
    Make the motors stop moving:            robot.stopMoving();
    Make the robot move forward:            robot.forward(<insert speed here>);
    Display a message to the screen:        telemetry.addData("Message","value");
    Update the screen for the message:      telemetry.update();
    Insert this after a robot.function:     runtime.reset();
    Determine how long the code has run:    runtime.seconds();
*/