#include <WPILib.h>
#include "XboxJoystickMap.h"
#include "SetablePIDOut.h"
#include "SetablePIDSource.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <String>

using namespace cv;
class Robot: public frc::IterativeRobot {
public:

	Joystick* controller;

	Talon* frontLeft;
	Talon* backLeft;
	Talon* frontRight;
	Talon* backRight;

	RobotDrive* driveTrain;

	Encoder* leftEnc;
	Encoder* rightEnc;
	CameraServer* cs;

	bool straight;
	bool open;


	cs::CvSink cvSink;
	cs::CvSource outputStreamStd;
	cv::Mat source;
	cv::Mat output;
	std::vector<Mat> contours;

	PIDController* pid;
	SetablePIDOut* pidout;
	SettablePIDSource* pidsrc;
	double diff = 0.0;
	double hLow, sLow, vLow, hHigh, sHigh, vHigh, p, i, d;
	double const SPEED_LIMIT = 0.8;
	double pidAdd = 0.275, pidoutput = 0.0;

	void RobotInit() {

		frontLeft = new Talon(1);
		backLeft = new Talon(0);
		frontRight = new Talon(3);
		backRight = new Talon(2);

		driveTrain = new RobotDrive(frontLeft, backLeft, frontRight, backRight);

		controller = new Joystick(0);

		cs = CameraServer::GetInstance();
		cs->AddAxisCamera("axis-camera.local");
		cvSink = cs->GetVideo();
		outputStreamStd = cs->PutVideo("Gray", 640, 480);

		leftEnc = new Encoder(0,1);
		rightEnc = new Encoder(2,3);
		open = false;

		pidout = new SetablePIDOut();
		pidsrc = new SettablePIDSource();
		pid = new PIDController(0.0005, 0.00002, 0.00095, pidsrc, pidout);
		}

	void AutonomousInit() override {
		straight = false;
	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		DashInit();
		pid->Enable();
	}

	void TeleopPeriodic() {

		//Update and receive Dash Values
		DashPeriodic();

		//Draw the largest rectangle to use on the screen
		Rect goal = DrawLargestRect();

		diff = (goal.width / 2 + goal.x) - 155;
		pidsrc->set(diff);

		//Drive robot
		double left = controller->GetRawAxis(XboxAxisLeftStickY);
		double right = controller->GetRawAxis(XboxAxisRightStickY);


		if (controller->GetRawButton(XboxButtonLeftBumper) && straight == false){
			straight = true;
		} else if (controller->GetRawButton(XboxButtonY) && (diff > 10 || diff < -10)){
			pidoutput = pidout->output < 0 ? pidout->output - pidAdd : pidout->output + pidAdd;
			driveTrain->TankDrive(-pidoutput, pidoutput);
		} else {
			DriveRowdyJr(left, right);
			pid->Reset();
			pid->Enable();
		}

	}

	void DriveRowdyJr(double left, double right){
		if(straight == true){
			printf("Encoders not implemented yet :( \n");
			driveTrain->TankDrive(-left * SPEED_LIMIT, -right * SPEED_LIMIT, true);
		} else {
			driveTrain->TankDrive(-left * SPEED_LIMIT, -right * SPEED_LIMIT, true);
		}
	}

	Rect DrawLargestRect(){
		//Grabs frame to view
		cvSink.GrabFrame(source);

		Rect biggest;

		//Converts frame to HSV Matrix
		SmartDashboard::PutBoolean("Empty source", source.empty());
		if (source.empty()){
			return biggest;
		}
		cvtColor(source, output, cv::COLOR_BGR2HSV);

		//Reduces image to colors inside bounds
		inRange(output, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), output);

		//Find continuous area shapes of bounded colors
		findContours(output.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_TC89_KCOS);

		//Find the largest area rectangle that surrounds the shape
		for(unsigned int i = 0; i < contours.size(); i++){
			Rect current = boundingRect(contours[i]);
			if(i == 0 || (current.width * current.height > biggest.width * biggest.height)){
				biggest = current;
			}
		}

		//Draw rectangle around largest shape
		rectangle(output, biggest, Scalar(255,255,255));



		//Post the output to the vidStream
		outputStreamStd.PutFrame(output);

		//Return the rectangle found
 		return biggest;
	}


	//Initialize SmartDashboard with necessary values and fields
	void DashInit(){
		//Put initial threshold values
		SmartDashboard::PutNumber("hLow", 55.0);
		SmartDashboard::PutNumber("sLow", 225.0);
		SmartDashboard::PutNumber("vLow", 75.0);
		SmartDashboard::PutNumber("hHigh", 90.0);
		SmartDashboard::PutNumber("sHigh", 255.0);
		SmartDashboard::PutNumber("vHigh", 255.0);
		SmartDashboard::PutNumber("P", pid->GetP());
		SmartDashboard::PutNumber("I", pid->GetI());
		SmartDashboard::PutNumber("D", pid->GetD());
		SmartDashboard::PutNumber("pidAdd", pidAdd);
	}

	//Update and use values on SmartDashboard
	void DashPeriodic(){
		hLow  = SmartDashboard::GetNumber("hLow", hLow);
		sLow  = SmartDashboard::GetNumber("sLow", sLow);
		vLow  = SmartDashboard::GetNumber("vLow", vLow);
		hHigh = SmartDashboard::GetNumber("hHigh", hHigh);
		sHigh = SmartDashboard::GetNumber("sHigh", sHigh);
		vHigh = SmartDashboard::GetNumber("vHigh", vHigh);
		pidAdd = SmartDashboard::GetNumber("pidAdd", pidAdd);


		SmartDashboard::PutNumber("Diff: ", diff);
		SmartDashboard::PutNumber("PIDOut", pidoutput);
		p = SmartDashboard::GetNumber("P", p);
		i = SmartDashboard::GetNumber("I", i);
		d = SmartDashboard::GetNumber("D", d);
		pid->SetPID(p, i, d);
	}


	void TestPeriodic() {

	}

private:
};

START_ROBOT_CLASS(Robot)
