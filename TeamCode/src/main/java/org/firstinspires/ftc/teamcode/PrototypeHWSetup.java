package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PrototypeHWSetup
{
    /* Public OpMode membersfkjebiofbwobfowebufeu. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  rightBackDrive      = null;
    public DcMotor  leftBackDrive        = null;
    public DcMotor  lift = null;
    public DcMotor  arm      = null;
    public DcMotor  arm2     = null;

    public CRServo mineralarm = null;
    public CRServo intake = null;
    public Servo door = null;
    public Servo sensorarm;
    /*   public DcMotor driveleft = null;
       public DcMotor driveright = null; */




    //public AnalogInput armsensor = null;
  //  public SensorDigitalTouch armsensor = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public PrototypeHWSetup(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive   = hwMap.dcMotor.get("left_front_drive");
        rightFrontDrive  = hwMap.dcMotor.get("right_front_drive");
        rightBackDrive = hwMap.dcMotor.get("right_back_drive");
        leftBackDrive = hwMap.dcMotor.get("left_back_drive");
        lift = hwMap.dcMotor.get("lift");
        arm = hwMap.dcMotor.get("arm");
        arm2 = hwMap.dcMotor.get("arm2");





        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.FORWARD);



        // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        lift.setPower(0);
        arm.setPower(0);
        arm2.setPower(0);

        intake   =hwMap.get(CRServo.class, "intake");
        door   =hwMap.get(Servo.class, "door");
        mineralarm   =hwMap.get(CRServo.class, "mineral_arm");
        sensorarm =hwMap.get(Servo.class, "sensor_arm");

        intake.setPower(0);
        door.setPosition(0);
        mineralarm.setPower(0);
        sensorarm.setPosition(.4);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      /*  driveright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */



        // Define and initialize ALL installed servos.



        

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
