package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AutonomousData;

public class Encoder {

    private DcMotor motor;   // Defines DC Motor Instance
    private int type;        // Define Either "NV20, NV40, NV60" motors
    private double AMHD_DIAMTER_INCHES;     // Defines AndyMark HD Mecanum Wheel Dimensions

    // Set Default Motor Tick Counts
    final static int NV20_COUNTS_PER_MOTOR_REV = 537;        // NeveRest 20 Motor Tick Count
    final static int NV40_COUNTS_PER_MOTOR_REV = 1120;       // NeveRest 40 Motor Tick Count
    final static int NV60_COUNTS_PER_MOTOR_REV = 1680;       // NeveRest 60 Motor Tick Count



    public Encoder(DcMotor motor, int type, double diam) {
        this.motor = motor;
        this.type = type;
        AMHD_DIAMTER_INCHES = diam;
    }


    public int ticksPerRev() {
        if (type == AutonomousData.NEVEREST_ENCODER20)
            return NV20_COUNTS_PER_MOTOR_REV;   // If NeveRest20 is passed to it, return NV20 tick count

        else if (type == AutonomousData.NEVEREST_ENCODER40)
            return NV40_COUNTS_PER_MOTOR_REV;   // If NeveRest40 is passed to it, return NV40 tick count

        else if (type == AutonomousData.NEVEREST_ENCODER60)
            return NV60_COUNTS_PER_MOTOR_REV;   // If NeveRest60 is passed to it, return NC60 tick count

        else
            return NV40_COUNTS_PER_MOTOR_REV;      // Default to NevRest40 motors
    }


    public int getEncoderCount() {
        return motor.getCurrentPosition();      //Reutnr current encoder Tick Count
    }

    public double motorRotations() {
        return (double) getEncoderCount() / ticksPerRev();      //Return calculated motor rotations
    }

    public double linDistance() {
        return motorRotations() * getWheelCircumfrence();       // Return calculated linDistance Value
    }

    public double getWheelCircumfrence() {
        return Math.PI * AMHD_DIAMTER_INCHES;                   // Return calcualted Wheel Circumfrance
    }


    // The Following methods are "Helper" methods which perform basic "reset" functions for Motor Encoder activities

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);      // Reset Motor Encoder
    }

    public void runWith() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);           // Set motor to use built-in Motor Encoder
    }

    public void runWithout() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);         // Set motor to NOT use built-in Motor Encoder
    }

    public void runToPosition() {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);             // Instruct motors to run to a desired position
    }

    public void setTarget(double linDistance) {
        int encoderPos = (int) (linDistance * ticksPerRev() / getWheelCircumfrence());
        motor.setTargetPosition(encoderPos);
    }

    public void setEncoderTarget(int encoderTicks) {
        motor.setTargetPosition(encoderTicks);                      // Send Encoder the "calculated ticks" to move
    }

    public void setup() {
        reset();
        runWith();
    }

}



