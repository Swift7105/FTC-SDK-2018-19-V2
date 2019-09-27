/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.ApplicationErrorReport;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class Gianna extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    private DcMotor rightback = null;
    private DcMotor leftintake = null;
    private DcMotor rightintake = null;
    private CRServo test = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftfront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightfront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftback = hardwareMap.get(DcMotor.class,"left_back_drive" );
        rightback = hardwareMap.get(DcMotor.class,"right_back_drive");
        leftintake = hardwareMap.get(DcMotor.class,"left_intake");
        rightintake = hardwareMap.get(DcMotor.class,"right_intake");
        test = hardwareMap.get(CRServo.class, "test");


        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        leftintake.setDirection(DcMotor.Direction.REVERSE);
        rightintake.setDirection(DcMotor.Direction.FORWARD);
        test.setDirection(CRServo.Direction.FORWARD);


        double mecanum = 0;
        double negmecanum = 0;
        double turning = 0;

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            mecanum = gamepad1.right_stick_y - gamepad1.right_stick_x;
            negmecanum = gamepad1.right_stick_y + gamepad1.right_stick_x;
            turning = gamepad1.left_stick_x;



            rightback.setPower(mecanum - turning);
            rightfront.setPower(negmecanum - turning);
            leftback.setPower(negmecanum + turning);
            leftfront.setPower(mecanum + turning);


          if(gamepad1.right_bumper){
                leftintake.setPower(1);
                rightintake.setPower(1);
            }
            else{
                leftintake.setPower(0);
                rightintake.setPower(0);
            }


            if(gamepad1.b){
                test.setPower(.5);
            }
            else{
                test.setPower(0);
            }



         //   telemetry.addData("Status", "Run Time: " + runtime.toString());
         //   telemetry.update();
        }
    }
}
