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

public class AutonomousBlueButtonAaron extends LinearOpMode
{
    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    LightSensor light;

    ColorSensor buttonColor;

    DcMotor motorCol;
    DcMotor motorPop;
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
        motorCol = hardwareMap.dcMotor.get("collector");
        motorPop = hardwareMap.dcMotor.get("popper");

        //motorUR.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);

        light.enableLed(true);
        //buttonColor.enableLed(true);
        //Movement

        motorPop.setPower(1);
        sleep(1500);
        motorPop.setPower(0);
        sleep(100);
        motorCol.setPower(1);
        sleep(3000);
        motorCol.setPower(0);
        sleep(1000);
        motorPop.setPower(1);
        sleep(1500);
        motorPop.setPower(0);
        sleep(500);
        mecaMovement(0,0);
        sleep(100);


        mecaMovement(315, 0.5);
        sleep(2000);
        mecaMovement(270, 0.5);
        sleep(500);
        mecaMovement(315, 0.5);
        sleep(1750);
        mecaMovement(180, 0.5);
        sleep(500);

        mecaMovement(270, 0.35);
        while (light.getLightDetected() <= 0.20)
        {
            //telemetry.addData("Light  ", light.getLightDetected());
            sleep(1);
        }

        telemetry.addData("Red  ", buttonColor.red());
        telemetry.addData("Blue ", buttonColor.blue());
        if(buttonColor.blue() > buttonColor.red())
        {
            mecaMovement(0,0.35);
            sleep(1000);
            mecaMovement(180, 0.35);
            sleep(400);
        }
        else{
            mecaMovement(90,0.35);
            sleep(800);
            if(buttonColor.blue() > buttonColor.red())
            {
                mecaMovement(0,0.35);
                sleep(1000);
                mecaMovement(180, 0.35);
                sleep(400);
            }
        }
        mecaMovement(180, 0.4);
        sleep(200);

        mecaMovement(270,0.5);
        sleep(2000);
        mecaMovement(0,0.35);
        sleep(1000);
        mecaMovement(180, 0.4);
        sleep(700);


        mecaMovement(270, 0.35);
        while (light.getLightDetected() <= 0.20)
        {
            //telemetry.addData("Light  ", light.getLightDetected());
            sleep(1);
        }

        telemetry.addData("Red  ", buttonColor.red());
        telemetry.addData("Blue ", buttonColor.blue());
        if(buttonColor.blue() > buttonColor.red())
        {
            mecaMovement(0,0.35);
            sleep(1000);
            mecaMovement(180, 0.35);
            sleep(400);
        }
        else{
            mecaMovement(90,0.35);
            sleep(800);
            if(buttonColor.blue() > buttonColor.red())
            {
                mecaMovement(0,0.35);
                sleep(1000);
                mecaMovement(180, 0.35);
                sleep(400);
            }
        }
        mecaMovement(180, 0.4);
        sleep(200);
        mecaMovement(0,0);
        telemetry.addLine("Did it work???");
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