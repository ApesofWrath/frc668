package frc.framework.controls;

public interface Controller {
    public boolean isButtonPressed(Button button);
    public double getAxis(Axis axis);
}
