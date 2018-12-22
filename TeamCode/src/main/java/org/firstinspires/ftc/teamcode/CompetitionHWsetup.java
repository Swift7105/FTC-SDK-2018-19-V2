package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

public class CompetitionHWsetup
{
    /* Public OpMode membersfkjebiofbwobfowebufeu. */
    public DcMotor  frontleftMotor   = null;
    public DcMotor  frontrightMotor  = null;
    public DcMotor  backleftMotor = null;
    public DcMotor  backrightMotor = null;
    public DcMotor  arm             = null;
     public DcMotor  lift            = null;
    public DcMotor  relicarm        = null;


    /*   public DcMotor driveleft = null;
       public DcMotor driveright = null; */
    public DcMotor  intakeright = null;
    public Servo    bigclaw = null;
    public Servo    frontclaw = null;
    public Servo    backclaw = null;
    public Servo    jewelarm = null;
    public Servo    jewelTwist = null;
    public Servo    relicclaw    = null;
    public Servo    SensorArm = null;


    public Sensor color_sensor = null;
    //public AnalogInput armsensor = null;
  //  public SensorDigitalTouch armsensor = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public CompetitionHWsetup(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontleftMotor   = hwMap.dcMotor.get("front_left_drive");
        frontrightMotor  = hwMap.dcMotor.get("front_right_drive");
        backleftMotor    = hwMap.dcMotor.get("back_left_drive");
        backrightMotor   = hwMap.dcMotor.get("back_right_drive");
      /*  driveleft = hwMap.dcMotor.get("drive_left");
        driveright = hwMap.dcMotor.get("drive_riht");*/
        arm        = hwMap.dcMotor.get("arm_motor");
        lift        = hwMap.dcMotor.get("lift_motor");
        intakeright = hwMap.dcMotor.get("intake_right");
        relicarm    = hwMap.dcMotor.get("relicarm");






        frontleftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontrightMotor.setDirection(DcMotor.Direction.FORWARD);
        backleftMotor.setDirection(DcMotor.Direction.REVERSE);
        backrightMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
    /*    driveright.setDirection(DcMotorSimple.Direction.FORWARD);
        driveleft.setDirection(DcMotorSimple.Direction.FORWARD); */

        intakeright.setDirection(DcMotor.Direction.FORWARD);
        relicarm.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        frontleftMotor.setPower(0);
        frontrightMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);
    /*    driveleft.setPower(0);
        driveright.setPower(0); */
        intakeright.setPower(0);
        arm.setPower(0);
        lift.setPower(0);
        relicarm.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        frontleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      /*  driveright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */
        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.

        bigclaw = hwMap.get(Servo.class, "bigclaw");
        frontclaw = hwMap.get(Servo.class, "frontclaw");
        backclaw = hwMap.get(Servo.class, "backclaw");
        jewelarm = hwMap.get(Servo.class, "barney");
        jewelTwist = hwMap.get(Servo.class, "jeweltwist");
        relicclaw   = hwMap.get(Servo.class, "relicclaw");
        SensorArm   =hwMap.get(Servo.class, "sensorarm");


        bigclaw.setPosition(0.5);
        frontclaw.setPosition(0.0);
        backclaw.setPosition(0.5);
        relicclaw.setPosition(1);
        SensorArm.setPosition(0.9);

        

       // armsensor = hwMap.get(SensorDigitalTouch.class, "armsensor");




    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
