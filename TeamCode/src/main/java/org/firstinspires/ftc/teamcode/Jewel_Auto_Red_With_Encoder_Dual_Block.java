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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Jewel Auto Red Encoder Dual Block", group="Pushbot")
@Disabled
public class Jewel_Auto_Red_With_Encoder_Dual_Block extends LinearOpMode {


    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /* Declare OpMode members. */
   CompetitionHWsetup robot = new CompetitionHWsetup();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {


        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AdaTTxT/////AAAAGZa0cW5OPUfNsaDTa3hXTXAVZzLmUtlM2vw1ea9hOvyg+YBpLFoEhYaqm5pdAUXUUXWi+vfLC8lTsa/FPWfqusPU4PTqqLE0Ojc6DWvH7NEI931kMAEfVBLxL+t5nyQDItuMEHfCRdCsLgbE71SPnxENwtX+3xP6p+hSw8Mx1rUjcIFug83wbhOYq2ERrbCqxWnbg63bjSdXLZofSZZlRvGKlDHvJKdKSLCGel5Ck6D0QBscg9CQExIFpT3n3OXdHKoTa+DgsF7y6TCEFeVEE1eVkxBr/mDJwGVGPllJAJVudtqt4MBNQsLAPWzUZTG6AOe2bgmjO/I5io5fByEbdknaaDUMMhBxnTLf3fGPh6lF";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");


        relicTrackables.activate();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled      = true;
        parameters1.loggingTag          = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        // Set up our telemetry dashboard
        composeTelemetry();


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");



        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //close claw
        robot.frontclaw.setPosition(0.95);
        robot.backclaw.setPosition(0.05);
        robot.bigclaw.setPosition(0.05);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();



        boolean looking = true;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        relicTrackables.activate();



        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();



// Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard


        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
              hsvValues);






        //Twist middle 0.68
        //Twist left 0.2
        //Twist Right .9
        //arm bottom is .3
        //arm up .9
        robot.jewelTwist.setPosition(.68);
        robot.jewelarm.setPosition(.9);
        sleep(500);

        robot.jewelarm.setPosition(.3);
        sleep(1000);

        //----------------------
        if (looking = true) {
            if (sensorColor.blue() > sensorColor.red()) {
                robot.jewelTwist.setPosition(0.2);
                sleep(500);
                robot.jewelTwist.setPosition(.68);
                robot.jewelarm.setPosition(0.9);
                sleep(500);
                looking = false;
            } else if (sensorColor.red() > sensorColor.blue()) {
                robot.jewelTwist.setPosition(0.9);
                sleep(500);
                robot.jewelarm.setPosition(0.9);
                sleep(1000);
                robot.jewelTwist.setPosition(.68);
                sleep(500);
                looking = false;
            }

            //--------------------------------

            sleep(500);
            robot.jewelTwist.setPosition(0.2);
            sleep(500);


            if (vuMark == RelicRecoveryVuMark.UNKNOWN) {

            }

            if (vuMark == RelicRecoveryVuMark.LEFT) {

                telemetry.addData("Status", "Going LEFT");
                //forward
                DriveEncoder(.25, -105, .25, -105);
                //turn
                DriveEncoder(.25, -50, .25, 50);
                //forward
                robot.backleftMotor.setPower(-0.5);
                robot.backrightMotor.setPower(-0.5);
                robot.frontleftMotor.setPower(-0.5);
                robot.frontrightMotor.setPower(-0.5);
                sleep(1000);
                robot.intakeright.setPower(.75);
                sleep(500);
                //backwards
                DriveEncoder(.25, 25, .25, 25);


                DriveEncoder(0.35,-100,0.35,100);
                robot.lift.setPower(0.5);
                sleep(500);
                robot.frontclaw.setPosition(0.01);
                robot.backclaw.setPosition(0.99);
                robot.bigclaw.setPosition(0.99);
                robot.intakeright.setPower(-0.75);
                DriveEncoder(0.25,-35,0.25,-35);
                robot.lift.setPower(-0.25);
                sleep(250);
                robot.frontclaw.setPosition(0.99);
                robot.backclaw.setPosition(0.01);
                robot.bigclaw.setPosition(0.01);
                DriveEncoder(0.35,-100,0.35,100);
                robot.backleftMotor.setPower(-0.5);
                robot.backrightMotor.setPower(-0.5);
                robot.frontleftMotor.setPower(-0.5);
                robot.frontrightMotor.setPower(-0.5);
                sleep(1000);
                robot.intakeright.setPower(.75);
                sleep(500);
                //back out
                DriveEncoder(.25,21,.25,21);
            }

            if (vuMark == RelicRecoveryVuMark.CENTER) {

                telemetry.addData("Status", "Going Center");
                //forward
                DriveEncoder(.25, -93, .25, -93);
                //turn
                DriveEncoder(.25, -55, .25, 55);
                //forward
                robot.backleftMotor.setPower(-0.5);
                robot.backrightMotor.setPower(-0.5);
                robot.frontleftMotor.setPower(-0.5);
                robot.frontrightMotor.setPower(-0.5);
                sleep(1000);
                robot.intakeright.setPower(.75);
                sleep(500);
                //backwards
                DriveEncoder(.25, 25, .25, 25);


                DriveEncoder(0.35,-100,0.35,100);
                robot.lift.setPower(0.5);
                sleep(500);
                robot.frontclaw.setPosition(0.01);
                robot.backclaw.setPosition(0.99);
                robot.bigclaw.setPosition(0.99);
                robot.intakeright.setPower(-0.75);
                DriveEncoder(0.25,-35,0.25,-35);
                robot.lift.setPower(-0.25);
                sleep(250);
                robot.frontclaw.setPosition(0.99);
                robot.backclaw.setPosition(0.01);
                robot.bigclaw.setPosition(0.01);
                DriveEncoder(0.35,-100,0.35,100);
                robot.backleftMotor.setPower(-0.5);
                robot.backrightMotor.setPower(-0.5);
                robot.frontleftMotor.setPower(-0.5);
                robot.frontrightMotor.setPower(-0.5);
                sleep(1000);
                robot.intakeright.setPower(.75);
                sleep(500);
                //back out
                DriveEncoder(.25,21,.25,21);
            }

            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("Status", "Going RIGHT");
                //forward
                DriveEncoder(.25, -67, .25, -67);
                //turn
                DriveEncoder(.25, -50, .25, 50);
                //forward
                robot.backleftMotor.setPower(-0.5);
                robot.backrightMotor.setPower(-0.5);
                robot.frontleftMotor.setPower(-0.5);
                robot.frontrightMotor.setPower(-0.5);
                sleep(1000);
                robot.intakeright.setPower(.75);
                sleep(500);
                DriveEncoder(.25, 25, .25, 25);


                DriveEncoder(0.35,-100,0.35,100);
                robot.lift.setPower(0.5);
                sleep(500);
                robot.frontclaw.setPosition(0.01);
                robot.backclaw.setPosition(0.99);
                robot.bigclaw.setPosition(0.99);
                robot.intakeright.setPower(-0.75);
                DriveEncoder(0.25,-35,0.25,-35);
                robot.lift.setPower(-0.25);
                sleep(250);
                robot.frontclaw.setPosition(0.99);
                robot.backclaw.setPosition(0.01);
                robot.bigclaw.setPosition(0.01);
                DriveEncoder(0.35,-100,0.35,100);
                robot.backleftMotor.setPower(-0.5);
                robot.backrightMotor.setPower(-0.5);
                robot.frontleftMotor.setPower(-0.5);
                robot.frontrightMotor.setPower(-0.5);
                sleep(1000);
                robot.intakeright.setPower(.75);
                sleep(500);
                //back out
                DriveEncoder(.25,21,.25,21);

            }


            // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


            telemetry.update();
            sleep(500);
        }
    }



    public void DrivePower (double leftpower, double rightpower){

        robot.frontrightMotor.setPower(rightpower);
        robot.backrightMotor.setPower(rightpower);
        robot.frontleftMotor.setPower(leftpower);
        robot.backleftMotor.setPower(leftpower);

    }



    //Stops all motors in the drivetrain
    public void DriveStop (){
        DrivePower(0,0);
    }


    //Allows the ability to run the Mechanum as a tank drive using the encoders to run to a spcific distance at a cetain speed.
    public void DriveEncoder (double leftpower, int leftdistance, double rightpower, int rightdistance){


        //sets the encoder values to zero
        robot.frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the position(distance) to drive to
        robot.frontrightMotor.setTargetPosition(rightdistance * 24);
        robot.backrightMotor.setTargetPosition(rightdistance * 24);
        robot.frontleftMotor.setTargetPosition(leftdistance * 24);
        robot.backleftMotor.setTargetPosition(leftdistance * 24);

        //engages the encoders to start tracking revolutions of the motor axel
        robot.frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //powers up the left and right side of the drivetrain independently
        DrivePower(leftpower, rightpower);

        //will pause the program until the motors have run to the previously specified position
        while (robot.frontrightMotor.isBusy() && robot.backrightMotor.isBusy() &&
                robot.frontleftMotor.isBusy() && robot.backleftMotor.isBusy())
        {

        }

        //stops the motors and sets them back to normal operation mode
        DriveStop();
        robot.frontrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backrightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}



