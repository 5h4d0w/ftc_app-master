
package com.qualcomm.ftcrobotcontroller.opmodes;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;
        //import com.qualcomm.robotcore.hardware.ColorSensor;


public class driveTest extends OpMode{

//<define any variables needed>

//<define motors and servos>
//

    DcMotorController.DeviceMode devMode;
    DcMotorController ctrlRight, ctrlLeft, ctrlAux;
    DcMotor bRight, bLeft, fRight, fLeft, climber;
    //ColorSensor colorSensor;

    //public enum ColorSensorDevice {MODERN_ROBOTICS_I2C};

    //public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;

    float lStick = 0, rStick = 0;
    boolean climb = false, revClimb = false;
//DcMotor <name>;

//Servo <name>;

     public driveTest() {

    }

    @Override
    public void init() {

        //maps the motors, defining what they're named on the robot controller
        bRight = hardwareMap.dcMotor.get("rightback");
        bLeft = hardwareMap.dcMotor.get("leftback");
        fRight = hardwareMap.dcMotor.get("rightfront");
        fLeft = hardwareMap.dcMotor.get("leftfront");
        climber = hardwareMap.dcMotor.get("climber");

        //maps the color sensor similar to the motors
        //colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //maps the motor controllers
        ctrlRight = hardwareMap.dcMotorController.get("right_controller");
        ctrlLeft = hardwareMap.dcMotorController.get("left_controller");
        ctrlAux = hardwareMap.dcMotorController.get("climber_controller");

        bRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        fRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        bLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        fLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


        //??????
        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

        //reverses the direction the left motors go by default so they go the same direction as the right
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override

    public void loop() {

        //declares the variables and assigned them the value of the left and right joystick
        lStick = gamepad1.left_stick_y;
        rStick = gamepad1.right_stick_y;

        //makes the dead zones for the joysticks and allows the robot to run depending on joystick inputs
        if (lStick > .1 || lStick < -.1) {
            bLeft.setPower(lStick);
            fLeft.setPower(lStick);
        }

        //sets power of the motors to 0 if they're not being used
        else {
            bLeft.setPower(0);
            fLeft.setPower(0);

        }
        if (rStick > .1 || rStick < -.1) {
            bRight.setPower(rStick);
            fRight.setPower(lStick);
        }
        else {
            bRight.setPower(0);
            fRight.setPower(0);
        }


        if(gamepad1.right_trigger > 0.1) {
            climber.setPower(gamepad1.right_trigger);
        }

        else if(gamepad1.left_trigger > 0.1) {
            climber.setPower(-gamepad1.left_trigger);
        }
        else {
            climber.setPower(0);
        }



    }

    @Override

    public void stop(){

    }

//this is used for making it so when the joysticks are at a low value it scales the

}