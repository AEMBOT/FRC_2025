package frc.robot.util;

public class BangBangController {
    public final Double deadzone;
    public Double setpoint;
    public Double position;
    public Double error;

    public BangBangController(Double deadzone) {
        this.deadzone = deadzone;
    }

    public void setSetpoint(Double setpoint) {
        this.setpoint = setpoint;
    }

    public Double update(Double position) {
        this.position = position;
        this.error = setpoint - position;
         if (error > deadzone) {
            return 1.0;
        } else if (error < -deadzone) {
            return -1.0;
        } else {
            return 0.0;
        }
    }
}
