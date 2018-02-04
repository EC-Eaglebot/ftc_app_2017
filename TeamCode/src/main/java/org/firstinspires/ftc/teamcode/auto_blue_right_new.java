package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name="Blue Team Right Stone JEWEL and BLOCK", group="Competition")
public class auto_blue_right_new extends LinearOpMode {

    // Declare OpMode members.
    HardwareClawbot     robot   = new HardwareClawbot();
    private ElapsedTime     runtime = new ElapsedTime();
    private boolean MoveOn = false;
    double go = HardwareClawbot.standardSpeed;
    private int stop = 500;
    VuforiaLocalizer vuforia;
    //HardwareClawbot.dir direction = robot.GetGlyphDirection(robot.cameraMonitorViewId);


    @Override
    public void runOpMode() {


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.init(hardwareMap);
        /*   for VuForia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ae0+l9f/////AAAAGSNVpVpt80F4p61FNmwiQSo+nRXp2HcThjA01Uak76AtdklG5SIElxoWuKmM04feEVJp1w1Bgmwq9ttjFbioiq30D/uRCucs90BxX6mAeMrjpCTWv8ySTyQw8Gse/t0OmnQzYlgMe+YJotsbVkiKWJtylDnXP7Lj621oWCH1CQx1vd6fqZ/CVP3AFj37Br/gxTXoyimhrgef4q0MIV4oo0MMDaRkhYEzfKY7qJcopSMKzsoHDFyjnnecUqDnYZAlU9AA/DI8UtnYJ7MoCnmZKS6xir8p6rTSem5Pm3613mBZc40JzVXWdsbtvbR9mfsG+Id1ZA4+q+to/uJCn8RHeWZRYdf7J3Uj6yPAw5SDk+ge";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        relicTrackables.activate();
        */

        double red_qualifier = 140;
        double arm_down = .8;
        double arm_middlepos = 0.42;
        String key_location = "unknown";


        //robot.stopMoving();
        //runtime.reset();

        //moves the paddle to initial middle position
        robot.paddle.setPosition(arm_middlepos);

        //moves the arm to the down position
        robot.arm.setPosition(.5);
        //close the claw here before the robot ever moves
        robot.closeClaws(runtime);
        //pause for a small time to slow the arm down
        sleep(1000);
        robot.arm.setPosition(.6);
        sleep(1000);
        robot.arm.setPosition(.7);
        sleep(1000);
        robot.arm.setPosition(.75);
        sleep(1000);
        robot.arm.setPosition(arm_down);
        MoveOn = true;
/*
//read the sensor
    runtime.reset();
    while (runtime.seconds() < 2) {
        if (vuMark != RelicRecoveryVuMark.CENTER && vuMark != RelicRecoveryVuMark.RIGHT && vuMark != RelicRecoveryVuMark.LEFT) {
            telemetry.addLine("Key still unknown");
            telemetry.update();
            key_location = "unknown";
        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            telemetry.addLine("Key is LEFT");
            telemetry.update();
            key_location = "left";
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addLine("Key is CENTER");
            telemetry.update();
            key_location = "center";
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addLine("Key is RIGHT");
            telemetry.update();
            key_location = "right";

        } }
        sleep(500);

 */
      runtime.reset();
      while (runtime.seconds() < 2 && MoveOn) {
            telemetry.addLine()
                    .addData("R", "%d", robot.color.red())
                    .addData("G", "%d", robot.color.green())
                    .addData("B", "%d", robot.color.blue());
            telemetry.update();
          if (robot.color.red() < red_qualifier){
              telemetry.addLine("BLUES CLUES");
              telemetry.update();
          } // if red
          else {
              telemetry.addLine("REDRUM REDRUM");
              telemetry.update();
          }
        }

        double power = .5;
        double twitchtime = 1.5;

        // Phase I: Read jewel color, knock off correct one
        // if blue
        if (robot.color.red() < red_qualifier){
            robot.paddle.setPosition(-1.0);
            telemetry.addLine("BLUES CLUES");
            telemetry.update();
            sleep(10000);
        } // if red
        else {
            robot.paddle.setPosition(1.0);
            telemetry.addLine("REDRUM REDRUM");
            telemetry.update();
            sleep(300);
        }
        robot.paddle.setPosition(arm_middlepos);
        robot.arm.setPosition(0.5);
        robot.arm.setPosition(0.2);
        robot.liftDrive.setPower(0.3);
        // runtime.reset();
        robot.forward(0.2, 1.6, runtime);
        sleep(stop);
        //runtime.reset();
        robot.spinRight(0.2, 0.45, runtime);
        sleep(stop);
        //runtime.reset();
        robot.forward(0.2, 1, runtime);
        sleep(stop);
        //sleep(stop);
        robot.forward(0.2, 0.5, runtime);
        sleep(stop);
        /*
        if (key_location.equals("left")) {
            robot.forward(0.2, 1.5, runtime);
            robot.turnRight(runtime);
        }
        else if (key_location.equals("center")) {
            robot.forward(0.2, 1.0, runtime);
            robot.turnRight(runtime);
        }
        else if (key_location.equals("right")) {
            robot.forward(0.2, 0.7, runtime);
            robot.turnRight(runtime);

        }*/

       /* code from the old wheel robot.paddle.setPosition(arm_middlepos);
        robot.arm.setPosition(.5);
        robot.liftDrive.setPower(0.3);
        runtime.reset();
        robot.forward(0.32, 1.5, runtime);
        runtime.reset();
        robot.spinLeft(0.2, 0.8, runtime);
        runtime.reset();
        robot.forward(0.2, 1, runtime);
        runtime.reset();
        robot.spinRight(0.05, 0.1, runtime);
        runtime.reset();
        robot.forward(0.2, 1, runtime);
        runtime.reset();
        */
       // robot.spinRight(0.05, 0.01, runtime);
        // runtime.reset();
        robot.forward(0.1, 0.8, runtime);
        runtime.reset();
        robot.openClaws(runtime);
        robot.forward(0.01, 0.5, runtime);
        robot.backward(0.1, .8, runtime);
        //runtime.reset();
        //robot.forward(0.1, 1, runtime);

       // robot.turnleft(runtime);


        telemetry.update();


        //robot.forward(0.5,2,runtime);
        //robot.strafeLeft(0.5,2,runtime);

      //  robot.strafeLeft(0.5,1.3,runtime);

        /*
        while (robot.direction == HardwareClawbot.dir.ERROR) {
            robot.GetGlyphDirection();
            telemetry.addData("Key", "still unknown");
        }
        if (robot.direction == HardwareClawbot.dir.LEFT) {
            telemetry.addData("Key", "is LEFT");
        }
        else if (robot.direction == HardwareClawbot.dir.CENTER) {
            telemetry.addData("Key", "is CENTER");
        }
        else if (robot.direction == HardwareClawbot.dir.RIGHT) {
            telemetry.addData("Key", "is RIGHT");
        }
*/


        //power = 1;
        // Phase II: Grab glyph, turn toward glyph wall, and move toward it
        //robot.closeClaws(runtime);
        //robot.forward(power,2,runtime);
        //robot.turnRight(runtime);
        //power = .25;
        //robot.forward(power,2.5,runtime);

        // End
        robot.arm.setPosition(.2);
        robot.stopMoving();



        //robot.forward(0.5, 1, runtime);
        //robot.stopMoving();
        // END

        // Below here is where the robot will stop and go to sleep. Make
        // sure you have the last line above as "robot.stop();" !

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
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