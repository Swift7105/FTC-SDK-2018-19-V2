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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MentorBot", group="Linear Opmode")
//@Disabled
public class MentorBot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private DcMotor arm = null;
    private DcMotor intake = null;

    private Servo wrist = null;
    private Servo claw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist.setPosition(0);
        claw.setPosition(.5);

        arm = hardwareMap.get(DcMotor.class, "arm");

        double mecanum = 0;
        double negmecanum = 0;
        double turning = 0;
        double wristpos;
        double armreset = 0;
        double armpos = 0;
        double armpospower = 0;
        double speed = 1;
        waitForStart();
        runtime.reset();
        armreset = arm.getCurrentPosition();

        while (opModeIsActive()) {

            armpos =(arm.getCurrentPosition() - armreset) * .214287;
            mecanum = gamepad1.right_stick_y + gamepad1.right_stick_x;
            negmecanum = gamepad1.right_stick_y - gamepad1.right_stick_x;
            mecanum = mecanum * speed;
            negmecanum = negmecanum * speed;
            turning = gamepad1.left_stick_x;

            lf.setPower(mecanum - turning);
            rf.setPower(negmecanum + turning);
            lb.setPower(negmecanum - turning);
            rb.setPower(mecanum + turning);

            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            if( Math.abs(gamepad2.right_stick_y) > 0  ){
                arm.setPower(gamepad2.right_stick_y / 2);
                armpospower = armpos;
            }
            else {

                arm.setPower(-(armpospower - armpos) * (armpospower - armpos) * (armpospower - armpos) * .001);
            }

            if(gamepad2.a){

                claw.setPosition(.5);
            }
            if(gamepad2.b){
                claw.setPosition(.05);
            }


            if(gamepad1.left_bumper){
                speed = .2;
            }
            else {
                speed = 1;
            }
            //test updated comment

            if(armpos > 94 ){



                wristpos =  (armpos - 85) * .004;
                wristpos = 1 - wristpos ;

                wrist.setPosition(wristpos);



            }
            else {
                wrist.setPosition(0);
            }

            telemetry.addData("arm position", armpos);
            telemetry.update();
        }
}