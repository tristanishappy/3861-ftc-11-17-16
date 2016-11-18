package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;

public class AutonomousRed extends LinearOpMode
{
    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    LightSensor light;

    ColorSensor buttonColor;

    Servo buttonServo;

    //DcMotor motorUL;
    //DcMotor motorUR;
    private double lift = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorRF = hardwareMap.dcMotor.get("motorRF");//sets DcMotors to type of motor
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        buttonColor = hardwareMap.colorSensor.get("color");

        light = hardwareMap.lightSensor.get("light");
        //motorUL = hardwareMap.dcMotor.get("motorUL");
        //motorUR = hardwareMap.dcMotor.get("motorUR");

        //motorUR.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);

        buttonServo = hardwareMap.servo.get("bServo");
        buttonServo.setPosition(180);
        light.enableLed(true);
        //buttonColor.enableLed(true);
        //Movement
        buttonServo.setPosition(-180);
        mecaMovement(45, 0.5);
        sleep(3250);
        mecaMovement(180, 0.25);
        sleep(550);
        mecaMovement(90,0.25);
        while(light.getLightDetected() <= 0.28)
        {
            sleep(5);
        }
        mecaMovement(0,0.2);
        sleep(500);
        mecaMovement(0,0);
        sleep(3000);
        mecaMovement(90,0.2);
        sleep(200);
        mecaMovement(0,0.2);
        sleep(1000);
        if(buttonColor.argb() <= 5)
        {
            mecaMovement(270,0.2);
            sleep(200);
            buttonServo.setPosition(180);
            mecaMovement(0,0.2);
            sleep(1500);
        }
        else{
            mecaMovement(180, 0.25);
            sleep(500);
            mecaMovement(270, 0.25);
            sleep(300);
            mecaMovement(0, 0.25);
            buttonServo.setPosition(180);
            sleep(1250);
            mecaMovement(0,0);
            sleep(2000);
        }
        mecaMovement(0,0);
        sleep(3000);

        mecaMovement(180, 0.25);
        sleep(750);
        mecaMovement(90,0.5);
        while(light.getLightDetected() <= 0.28)
        {
            sleep(5);
        }
        mecaMovement(0,1);
        sleep(1000);
        if(buttonColor.argb() >= 5)
        {
            buttonServo.setPosition(180);
        }
        else{
            mecaMovement(270, 1);
            sleep(1000);
            mecaMovement(0,0);
            buttonServo.setPosition(180);
            sleep(2000);
        }
        mecaMovement(0,0);
    }

    public void mecaMovement(double degrees, double power)
    {
        double radians = Math.toRadians(degrees+45);
        motorLF.setPower(power * Math.cos(radians));
        motorLB.setPower(power * Math.sin(radians));
        motorRF.setPower(power * Math.sin(radians));
        motorRB.setPower(power * Math.cos(radians));
    }
    public void movement(double powerL, double powerR)
    {
        motorLF.setPower(powerL);
        motorLB.setPower(powerL);
        motorRF.setPower(powerR);
        motorRB.setPower(powerR);
    }
}