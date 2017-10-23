package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/**
 * Created by student on 9/23/17.
 */


public class testop extends OpMode{


    public DcMotor dc_drive_left = null;
    public DcMotor dc_drive_right= null;
    public DcMotor dc_drive_controller = null;

    public void init() {
        dc_drive_controller = hardwareMap.dcMotor.get("drive_controller");
        dc_drive_left = hardwareMap.dcMotor.get("left drive");
        dc_drive_right = hardwareMap.dcMotor.get("right drive");

    }
    public void loop() {
        dc_drive_left.setPower(gamepad1.left_stick_y);
        dc_drive_right.setPower(gamepad1.right_stick_y);
    }



}
