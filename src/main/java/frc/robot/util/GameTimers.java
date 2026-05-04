package frc.robot.util;

import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleTopic;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.smartdashboard.SmartDashboard;

public final class GameTimers {

	//private static final DoubleTopic shiftTimeTopic =
	//		NetworkTableInstance.getDefault().getDoubleTopic("/Elastic/Match/ShiftTimeSeconds");
	//private static final DoublePublisher shiftTimePublisher =
	//		shiftTimeTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

	private static final double[] changeTimes = {130.0, 105.0, 80.0, 55.0, 30.0, 0.0};

	private GameTimers() {
		throw new UnsupportedOperationException("Static utility class ");
	}

	public static void update() {
		double matchTime = getMatchTime();
		double shiftTime = shiftTimeLeft(matchTime);

		SmartDashboard.putNumber("Match/TimeRemaining", matchTime);
		SmartDashboard.putNumber("Match/ShiftTimeRemaining", shiftTime);
		//shiftTimePublisher.set(shiftTime);
	}

	private static double getMatchTime() { //counts down in practice and at comp, in tele or auto modes counts up from 0
		return Math.max(0.0, DriverStation.getMatchTime());
	}

	private static double shiftTimeLeft(double matchTime) {
		if (matchTime <= 0.0) {
			return 0.0;
		}

		if (!DriverStation.isTeleopEnabled()) {
			return matchTime;
		}

		for (double shiftEnd : changeTimes) {
			if (matchTime > shiftEnd) {
				return Math.max(0.0, matchTime - shiftEnd);
			}
		}

		return 0.0;
	}
}
