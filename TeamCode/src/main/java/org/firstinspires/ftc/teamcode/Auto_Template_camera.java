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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


@Autonomous(name="Pushbot: Crater Side", group="Pushbot")
//@Disabled
public class Auto_Template_camera extends LinearOpMode {

    /* Declare OpMode members. */
    PrototypeHWSetup robot = new PrototypeHWSetup();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AdaTTxT/////AAAAGZa0cW5OPUfNsaDTa3hXTXAVZzLmUtlM2vw1ea9hOvyg+YBpLFoEhYaqm5pdAUXUUXWi+vfLC8lTsa/FPWfqusPU4PTqqLE0Ojc6DWvH7NEI931kMAEfVBLxL+t5nyQDItuMEHfCRdCsLgbE71SPnxENwtX+3xP6p+hSw8Mx1rUjcIFug83wbhOYq2ERrbCqxWnbg63bjSdXLZofSZZlRvGKlDHvJKdKSLCGel5Ck6D0QBscg9CQExIFpT3n3OXdHKoTa+DgsF7y6TCEFeVEE1eVkxBr/mDJwGVGPllJAJVudtqt4MBNQsLAPWzUZTG6AOe2bgmjO/I5io5fByEbdknaaDUMMhBxnTLf3fGPh6lF";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private boolean loop = TRUE;

    private int cubepos = -1;

    double timerreset = 0;

    @Override public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        robot.init(hardwareMap);
        robot.leds.setPower(0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        if (tfod != null) {
            tfod.activate();
        }
        if (tfod != null) {
            tfod.activate();
        }


        waitForStart();



        timerreset = getRuntime();
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
/*
        robot.lift.setPower(-1);
        sleep(2100);
        robot.lift.setPower(-.1);
        DriveForward(.7,9,  .7,9);
        robot.lift.setPower(0);
        DriveStrafe(.9,14,.9,-14);
        DriveForward(.9,-17,  .9,-17);
        DriveForward(.5,6,  .5,-6);
*/

     //   if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
        robot.lift.setPower(-1);
        sleep(2100);
        robot.lift.setPower(-.1);

            while (loop == TRUE) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

     /*                   if (updatedRecognitions.size() < 2){
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getTop();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getTop();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1){
                                telemetry.addData("Gold Mineral Position", "Right");
                                cubepos = 2;
                                loop = FALSE;
                            }
                        }*/

                         if (updatedRecognitions.size() > 0) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getTop();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getTop();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                            if (goldMineralX != -1){
                                if (goldMineralX > 650){
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    loop = FALSE;
                                    cubepos = 1;
                                }
                                else{
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    loop = FALSE;
                                    cubepos = 0;
                                }
                            }
                            else{
                                telemetry.addData("Gold Mineral Position", "Right");
                                cubepos = 2;
                                loop = FALSE;
                            }
/*
                            if (goldMineralX != -1 && silverMineral1X != -1){
                                if (goldMineralX > silverMineral1X){
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    loop = FALSE;
                                    cubepos = 1;
                                }
                                else{
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    loop = FALSE;
                                    cubepos = 0;
                                }
                            }

                            if (silverMineral1X != -1 && silverMineral2X != -1){
                                telemetry.addData("Gold Mineral Position", "Right");
                                cubepos = 2;
                                loop = FALSE;
                            }*/
                        }

                        if (getRuntime() - timerreset > 5){
                            loop = FALSE;
                            cubepos = 1;
                        }
                        telemetry.update();
                    }
                }
            }

        if (tfod != null) {
            tfod.shutdown();
        }

        robot.leds.setPower(1);

        DriveForward(.7,9,  .7,9);
        robot.lift.setPower(0);
        DriveStrafe(.9,14,.9,-14);

     //   DriveForward(.5,-6,  .5,6);

        if (cubepos == 0){

            DriveForward(1, 18, 1, -18);
            DriveStrafe(.9,72,.9,-72);
            DriveStrafe(.9,-30,.9,30);
            DriveForward(.7, -18, .7, 18);
            DriveForward(1, 70, 1, 70);

        }
        if (cubepos == 1){
            DriveForward(1,-10,  1,-10);
            DriveStrafe(1,64,1,-64);
            DriveStrafe(.9,-32,.9,32);
            DriveForward(1,95,  1,95);

        }
        if (cubepos == 2) {
            DriveForward(.9, -27, .9, 27);
            DriveStrafe(.9,78,.9,-78);
            DriveStrafe(.9,-34,.9,34);
            DriveForward(.7, 27, .7, -27);
            DriveForward(1, 128, 1, 128);

        }
/*
        DriveForward(.7,24,  .7,-24);
        DriveStrafeTime(70,-70,  1);
        DriveForward(1,120,  1,120);
*/
        DriveForward(.7,27,  .7,-27);
        DriveStrafe(.9,18,.9,-18);
        DriveForward(.9,105,  .9,105);

        robot.intake.setPower(.3);
        sleep( 2000);
        robot.intake.setPower(0);

        robot.intake.setPower(0);
        robot.arm.setPower(-.8);
        robot.arm2.setPower(-.8);
        sleep( 800);

        robot.arm.setPower(0);
        robot.arm2.setPower(0);
        sleep( 100);

        robot.arm.setPower(.8);
        robot.arm2.setPower(.8);
        sleep( 500);
        robot.arm.setPower(0);
        robot.arm2.setPower(0);
        sleep( 100);

        robot.mineralarm.setPower(1);
        DriveForward(1,-100,  1,-100);
        DriveStrafe(.9,-20,.9,20);
        DriveForward(.9,25,  .9,-25);
        DriveStrafe(.9,-100,.9,100);


        robot.intake.setPower(-1);
        robot.arm.setPower(-.5);
        robot.arm2.setPower(-.5);
        sleep( 1600);
        robot.arm.setPower(0);
        robot.arm2.setPower(0);
        robot.lift.setPower(.7);
        sleep( 2600);
        robot.lift.setPower(0);
        DriveForward(.9,15,  .9,15);
        DriveForward(.9,-15,  .9,-15);
        robot.intake.setPower(0);
        robot.mineralarm.setPower(0);



        telemetry.update();
        sleep(500);
    }




    public void DrivePower (double leftpower, double rightpower) {

        robot.rightFrontDrive.setPower(rightpower);
        robot.rightBackDrive.setPower(rightpower);
        robot.leftFrontDrive.setPower(leftpower);
        robot.leftBackDrive.setPower(leftpower);

    }

    //Stops all motors in the drivetrain
    public void DriveStop (){
        DrivePower(0,0);
    }


    //Allows the ability to run the Mechanum as a tank drive using the encoders to run to a spcific distance at a cetain speed.
    public void DriveForward (double leftpower, int leftdistance, double rightpower, int rightdistance){

        //sets the encoder values to zero
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the position(distance) to drive to
        robot.rightFrontDrive.setTargetPosition(rightdistance * 35);
        robot.rightBackDrive.setTargetPosition(rightdistance * 35);
        robot.leftFrontDrive.setTargetPosition(leftdistance * 35);
        robot.leftBackDrive.setTargetPosition(leftdistance * 35);

        //engages the encoders to start tracking revolutions of the motor axel
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //powers up the left and right side of the drivetrain independently
        DrivePower(leftpower, rightpower);

        //will pause the program until the motors have run to the previously specified position
        while (robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy() &&
                robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy())
        {

        }

        //stops the motors and sets them back to normal operation mode
        DriveStop();
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Allows the ability to run the Mechanum as a tank drive using the encoders to run to a spcific distance at a cetain speed.
    public void DriveStrafe (double leftpower, int leftdistance, double rightpower, int rightdistance){


        //sets the encoder values to zero
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the position(distance) to drive to
        robot.rightFrontDrive.setTargetPosition(rightdistance * 35);
        robot.rightBackDrive.setTargetPosition(leftdistance * 35);
        robot.leftFrontDrive.setTargetPosition(leftdistance * 35);
        robot.leftBackDrive.setTargetPosition(rightdistance * 35);

        //engages the encoders to start tracking revolutions of the motor axel
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //powers up the left and right side of the drivetrain independently
        DrivePower(leftpower, rightpower);

        //will pause the program until the motors have run to the previously specified position
        while (robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy() &&
                robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy())
        {

        }

        //stops the motors and sets them back to normal operation mode
        DriveStop();
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveStrafeTime (double leftpower,double rightpower, long timetime){


        //engages the encoders to start tracking revolutions of the motor axel
        robot.rightFrontDrive.setPower(rightpower);
        robot.rightBackDrive.setPower(leftpower);
        robot.leftFrontDrive.setPower(leftpower);
        robot.leftBackDrive.setPower(rightpower);

        sleep(1000 * timetime);

        //stops the motors and sets them back to normal operation mode
        DriveStop();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}








