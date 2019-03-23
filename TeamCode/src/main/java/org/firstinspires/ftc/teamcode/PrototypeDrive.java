/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


@TeleOp(name="Pushbot: PrototypeDrive", group="Pushbot")
//@Disabled
public class PrototypeDrive extends OpMode{
   /* int relicdistace;
      boolean relicclaw;
    /* Declare OpMode members. */
    PrototypeHWSetup robot = new PrototypeHWSetup();// use the class created to define a Pushbot's hardware
     double reverse = 1;
     double speed = 1;
     double turning;
     double mecanum;
     double negmecanum;
     double reversetimer;
     double speedturning = 0;
     double inittime = 0;
     double armspeed = 0;
     double armreset = 0;
     boolean bbep = FALSE;
     double armpos = 0;

    double ledbrightness = 1;
     double ledmultiplier = 1;

     double gravcomp = 0.05;
     //----------------------------------------------------------------------
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle = 0;
    double global90 = 0;
  //  double Xposition = 0;
  //  double Yposition = 0;
//----------------------------------------------------------------------


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        armreset = robot.arm2.getCurrentPosition();

       // robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  relicclaw = false;

//----------------------------------------------------------------------


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
      /*  while (!imu.isGyroCalibrated()) {

        }
        */
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
//----------------------------------------------------------------------


        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
    }
    @Override
    public void init_loop() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }
    /*
     * Code to run ONCE when the driver hits PLAY
     *
     */




    @Override
    public void start() { }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {

        getAngle();
        turning = gamepad1.left_stick_x * .75;
        negmecanum = gamepad1.right_stick_y - gamepad1.right_stick_x;
        mecanum = gamepad1.right_stick_y + gamepad1.right_stick_x;
   //     armpos = (robot.arm2.getCurrentPosition() / 35) - armposreset;
       // Yposition += (-gamepad1.right_stick_y * Math.cos(Math.abs(globalAngle))) + (gamepad1.right_stick_x * Math.sin(Math.abs(globalAngle)));
       // Xposition += (gamepad1.right_stick_x * Math.sin(Math.abs(globalAngle))) + (-gamepad1.right_stick_y * Math.cos(Math.abs(globalAngle)));


     //   Xposition += (gamepad1.right_stick_x * Math.cos(globalAngle));

        /*
        if (gamepad1.dpad_up) {
            getAngle();
            if (globalAngle > 0){
                turning += ((globalAngle / 50) * .5) + .2;
            }
            else{
                turning -= ((-globalAngle / 50) * .5) + .2;
            }
        }
*/

        if (ledbrightness < -1){
            ledbrightness = 1;
        }
        else {
            ledbrightness -= .05 * ledmultiplier;
            if (ledbrightness < 0){
            //    robot.leds.setPower(-ledbrightness);
            }
            else {
             //   robot.leds.setPower(ledbrightness);
            }
        }

        if (gamepad1.right_bumper){
            if (reversetimer > 10){
                if (globalAngle > 1){
                    turning += ((globalAngle + (gamepad1.left_trigger - .9))  / 40) + .07;
                }
                else if (globalAngle < -1 ){
                    turning -= ((-globalAngle + (gamepad1.left_trigger - .9)) / 40) + .07;
                }
                else{

                }
            }
            reverse = -1;
            reversetimer += 1;
        }
        else {
            reverse = 1;
            reversetimer = 0;
        }

        global90 = globalAngle + 95;

        if (gamepad1.right_trigger > .5){
            if (global90 > 1){
                turning += ((global90 + (gamepad1.left_trigger - .9))  / 40) + .07;

           //     turning += (global90 * global90 * global90 / 729000);
            }
            else if ((globalAngle + 90) < -1 ){
                turning -= ((-global90 + (gamepad1.left_trigger - .9)) / 40) + .07;

          //      turning -= (-global90 * -global90 * -global90 / 729000 );
            }
            else{

            }
        }
        Range.clip(turning,-1,1);


        if (gamepad1.left_bumper){
            speedturning = .45;
            speed = .5;

        }
        else {
            speed = 1;
            speedturning = 1;
        }


        robot.leftFrontDrive.setPower(((negmecanum)* reverse)* speed  - (turning * speedturning)) ;
        robot.rightBackDrive.setPower(((negmecanum)* reverse)* speed  + (turning * speedturning));
        robot.rightFrontDrive.setPower(((mecanum)* reverse)* speed + (turning * speedturning));
        robot.leftBackDrive.setPower(((mecanum) * reverse)* speed - (turning * speedturning));



        if (Math.abs(gamepad2.right_stick_y) > .1) {
            armspeed = robot.arm2.getCurrentPosition() - armreset;
            armspeed = Math.abs(armspeed);
            armspeed = 2800 - armspeed;
            armspeed = armspeed / 2800;
            robot.arm.setPower(gamepad2.right_stick_y * -armspeed);
            robot.arm2.setPower(gamepad2.right_stick_y * -armspeed);
        }
        else if(gamepad2.right_trigger > .5){
            robot.arm.setPower(-.3);
            robot.arm2.setPower(-.3);
        }
        else{

            armreset = robot.arm2.getCurrentPosition();

            robot.arm.setPower(0);
            robot.arm2.setPower(0);
        }


        /*
        if (Math.abs(gamepad2.right_stick_y) > .5){
            armreset += (Math.abs(gamepad2.right_stick_y) / gamepad2.right_stick_y) * 30;
        }
        else{
            armreset = robot.arm2.getCurrentPosition();
        }
        armpos = robot.arm2.getCurrentPosition() - armreset ;
        armpos = armpos / 300 ;
        armspeed = armpos * armpos * armpos;
        robot.arm.setPower(armspeed);
        robot.arm2.setPower(armspeed);
*/

 /*       if (gamepad2.b){
            if (bbep == FALSE){
                armreset += 1;
                bbep = TRUE;
            }
        }
        else {
            bbep = FALSE;
        }


        armpos = robot.arm2.getCurrentPosition() / 35 ;
        armpos = armpos + armreset;
        if (gamepad2.right_stick_y > 0){

            armspeed = (65 - armpos) / 50;
          //  armpos = armpos - 60;
          //  armspeed = armpos * armpos * armpos;
            robot.arm.setPower(-gamepad2.right_stick_y * armspeed);
            robot.arm2.setPower(-gamepad2.right_stick_y * armspeed);

        }
        else if (gamepad2.right_stick_y < 0){

            armspeed = armpos / 50;
            //armspeed = armpos ;
            //armspeed = armspeed * armspeed * armspeed * .00008;
           // armpos = armpos - 15;
            //armspeed = armpos * armpos * armpos;
            robot.arm.setPower(-gamepad2.right_stick_y * armspeed);
            robot.arm2.setPower(-gamepad2.right_stick_y * armspeed);

        }

        else{
            robot.arm.setPower(0);
            robot.arm2.setPower(0);
        } */

 /*
        robot.arm.setPower(gamepad2.right_stick_y * armspeed);
        robot.arm2.setPower(gamepad2.right_stick_y * armspeed);

        if ( gamepad2.left_stick_y < 0){
            robot.intake.setPower(-gamepad2.left_stick_y * .2);

        }
        else{
            if(armpos < 28){
                robot.intake.setPower(-gamepad2.left_stick_y * .4);

            }
            else{
                robot.intake.setPower(-gamepad2.left_stick_y);

            }
        }*/
        robot.intake.setPower(-gamepad2.left_stick_y);


        /*if(gamepad2.dpad_up){
            robot.intake.setPower(.8);

        }
        else if (gamepad2.dpad_down){
            robot.intake.setPower(-.9);
        }
        else {
            robot.intake.setPower(0);
        }*/

        if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1){
            if (inittime == 0){
                inittime = getRuntime();

            }
        }


        if (gamepad2.left_trigger > .5){
            robot.lift.setPower(1);
        }
        else {
            if ((getRuntime() - inittime) > 108 && (getRuntime() - inittime) < 111){
                robot.lift.setPower(-1);
                ledmultiplier = 10;
            }
            else{
                robot.lift.setPower(0);
            }
        }




        if (gamepad2.a){
            robot.mineralarm.setPower(1);
        }
        else if (gamepad2.x){
            robot.mineralarm.setPower(-1);
        }
        else {
            robot.mineralarm.setPower(0);
        }

        if (gamepad2.left_bumper){
            robot.lift.setPower(-1);
        }

        if (gamepad2.right_bumper){
            //open
            robot.door.setPosition(0.05);
        }
        else {
            //closed
            robot.door.setPosition(.3);
        }




/*
        if (gamepad1.right_bumper){
            if (reversem == 1) {
                reversem = -1;
                if (reverse > 0){
                    reverse = -1;
                }
                else {
                    reverse = 1;
                }
            }

        }
        else {
            reversem = 1;
        }
*/

        if (gamepad1.x){
            resetAngle();
     //       Xposition = 0;
            //Yposition = 0;
        }



//----------------------------------------------------------------------
        telemetry.addData(" arm", robot.arm2.getCurrentPosition());
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("trigger", gamepad2.right_trigger);
        telemetry.addData("gamestick", gamepad2.right_stick_y);
       // telemetry.addData("sticky", robot.touchsensor);

        telemetry.update();




    /*    if (robot.arm.getCurrentPosition() < 415){
            telemetry.addData("angle", robot.arm.getCurrentPosition());

        } */
//----------------------------------------------------------------------
    }


    //----------------------------------------------------------------------
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    //----------------------------------------------------------------------

    @Override
    public void stop() {
    }

}
