package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;

import edu.wpi.first.math.Pair;

public class ArmAimHelper {
    // list of observed targetYs and setpoints
    // this lets us collect different distances, so we aren't stuck with a straight line
    private final ArrayList<ArmSetpoint> m_armSetpoints = new ArrayList<ArmSetpoint>() {
        {
            add(new ArmSetpoint(0, 4.0, 1)); // 0 m
            add(new ArmSetpoint(-16.7, 12.78, 1)); // 1m
            add(new ArmSetpoint(-25, 18.0, 1)); // Podium
            add(new ArmSetpoint(-26, 18.66, 1)); // 2m
            add(new ArmSetpoint(-30.18, 20.35, 1)); // 3m
            add(new ArmSetpoint(-31.64, 21.0, 1)); // 3.5m
            add(new ArmSetpoint(-33.14, 21.0, 1)); // 4m
        }
    };

    public Pair<Double, Double> getArmSetpoint(TagLimelight limelight) {
        var targetY = limelight.getTargetY();
        var setpoints = m_armSetpoints.stream()
                .sorted(new ArmSetpointComparator(targetY)) // sort by which setpoints are closer to the distance we a looking for 
                .limit(2) // grab just the first two
                .toList();
        
        var setpoint1 = setpoints.get(0);
        var setpoint2 = setpoints.get(1);

        // our targetY is what ratio between these two setpoints?
        var distRatio = (targetY - setpoint1.TargetY) / (setpoint2.TargetY - setpoint1.TargetY);

        // apply that ratio to the setpoints range
        var setpoint = setpoint1.Setpoint + ((setpoint2.Setpoint - setpoint1.Setpoint) * distRatio);
        var speed = setpoint1.Speed + ((setpoint2.Speed - setpoint1.Speed) * distRatio);

        return new Pair<Double,Double>(setpoint, speed);
    }

    // tracks a distance/setpoint pair
    private class ArmSetpoint {
        public ArmSetpoint(double targetY, double setpoint, double speed) {
            TargetY = targetY;
            Setpoint = setpoint;
            Speed = speed;
        }

        public double TargetY;
        public double Setpoint;
        public double Speed;
    }

    // compares two ArmSetpoint by how close it is to our target distance
    private class ArmSetpointComparator implements Comparator<ArmSetpoint> {
        private double targetY;

        public ArmSetpointComparator(double targetY) {
            this.targetY = targetY;
        }

        @Override
        public int compare(ArmSetpoint a, ArmSetpoint b) {
            var aProximity = Math.abs(a.TargetY - targetY);
            var bProximity = Math.abs(b.TargetY - targetY);
            return Double.compare(aProximity, bProximity);
        }

    }
}
