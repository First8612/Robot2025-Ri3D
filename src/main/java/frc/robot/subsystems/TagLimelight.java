package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TargetType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TagLimelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-tags");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    private int m_lastAprilTag = 0;

    public double getTargetX() {
        return tx.getDouble(0);
    }

    public double getTargetY() {
        return ty.getDouble(0);
    }

    public double getTargetDistBlended() {
        return (getTargetY() + getTargetA()) / 2;
    }

    public double getTargetDist() {
        var y = getTargetY();
        if (y == -1) return 0;

        var ty = -y;

        if (ty < 26) { // 2m
            
        } 
        else if (ty < 32.75) { // 4m

        }
        else if (ty < 34.5) { // 6m

        }

        return (ty / 34.5) * 6;
    }

    public double getTargetA() {
        return ta.getDouble(0);
    }

    public int getAprilTag() {
        return (int)tid.getInteger(0);
    }

    public int getLastAprilTag() {
        return m_lastAprilTag;
    }

    public TargetType getLastTargetType() {
        switch (m_lastAprilTag) {
            case 5:
            case 6:
                return TargetType.AMP;

            case 4:
            case 7:
                return TargetType.SPEAKER;

            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
                return TargetType.STAGE;
    
            default:
                return TargetType.NONE;
        }
    }

    public void periodic() {
        var aprilTag = getAprilTag();
        if (aprilTag != 0) {
            m_lastAprilTag = aprilTag;
        }

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard
        SmartDashboard.putNumber("Tag Limelight X", x);
        SmartDashboard.putNumber("Tag Limelight Y", y);
        SmartDashboard.putNumber("Tag Limelight Dist", getTargetDist());
        SmartDashboard.putNumber("Tag DistBlended", getTargetDistBlended());
        SmartDashboard.putNumber("Tag Limelight ID", getAprilTag());
        SmartDashboard.putNumber("Tag Limelight Area", area);
        SmartDashboard.putString("Tag Limelight LastTargetType", getLastTargetType().toString());
    }
}
