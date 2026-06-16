package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Util;

public class PoseFollower {

    private double translate_kP = 3.0;
    private double rotate_kP = 2.0;

    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private Pose2d targetPose = new Pose2d();

    public PoseFollower(Pose2d targetPose, double maxSpeed, double translate_kp, double rotate_kP) {
        setParams(targetPose, maxSpeed, translate_kp, rotate_kP);
    }

    public PoseFollower(Pose2d targetPose, double maxSpeed) {
        this(targetPose, maxSpeed, 3.0, 2.0);
    }

    public void setParams(Pose2d targetPose, double maxSpeed){
        setParams(targetPose, maxSpeed, translate_kP, rotate_kP);
    }

    public void setParams(Pose2d targetPose, double maxSpeed, double translate_kp, double rotate_kP){
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.translate_kP = translate_kp;
        this.rotate_kP = rotate_kP;
    }

    public ChassisSpeeds process() {
        Transform2d poseDiff = targetPose.minus(Drive.getInstance().getPose()); // Calculate the difference in position.

        double magnitude = Math.hypot(
                poseDiff.getX(),
                poseDiff.getY()); // Calculate the magnitude (distance to the target).

        Transform2d direction; // This will hold the normalized and limited vector.

        Rotation2d rotationDiff = poseDiff.getRotation();
        double radDiff = rotationDiff.getRadians();
        double angularVelocity = Util.limit(rotate_kP * radDiff, -Drive.getInstance().MAX_ANGULAR_VEL_radps,
                Drive.getInstance().MAX_ANGULAR_VEL_radps);

        angularVelocity *= Units.radiansToDegrees(1);

        double output = Util.limit(translate_kP * magnitude, maxSpeed);
        if(magnitude < 0.05){
            output = 0;
        }

        direction = poseDiff.times(output/magnitude);

        // Create the ChassisSpeeds with the limited linear velocities.
        ChassisSpeeds speeds = new ChassisSpeeds(
                direction.getX(), // Linear velocity in the x-direction (vx)
                direction.getY(), // Linear velocity in the y-direction (vy)
                angularVelocity // No angular velocity (omega)
        );

        return speeds;
    }

}