package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteLimelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    public double getTargetX() {
        return tx.getDouble(0);
    }

    public double getTargetY() {
        return ty.getDouble(0);
    }

    public double getTargetDistBlended() {
        return (getTargetY() + getTargetA()) / 2;
    }

    public double getTargetA() {
        return ta.getDouble(0);
    }

    public void periodic() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard
        SmartDashboard.putNumber("Note Limelight X", x);
        SmartDashboard.putNumber("Note Limelight Y", y);
        SmartDashboard.putNumber("Note DistBlended", getTargetDistBlended());
        SmartDashboard.putNumber("Note Limelight Area", area);
    }
}
