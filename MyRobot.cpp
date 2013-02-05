                                                                     
                                                                     
                                                                     
                                             
#include <math.h>

#include "WPILib.h"
#include "nivision.h"

class BinaryImageWrapper: public BinaryImage {
	public: BinaryImageWrapper(Image* wrappedImage) {
		m_imaqImage = wrappedImage;
	}
	~BinaryImageWrapper() {
	}
};

/**s
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *myRobot; // robot drive system
	Joystick *rightstick; // only right joystick
	Joystick *leftstick; //only left
	Joystick *lonelystick; //third joystick
    Jaguar *Motor1;
    Jaguar *Motor2;
    Jaguar *Motor3;
    Jaguar *Motor4;
    Jaguar *BallGathererMotor9;//ball gatherer
    Solenoid *sol;
    AxisCamera *cam;
    HSLImage *img;
    Relay *rlyLED; //LED lights
    Relay *kickermotor;
    DriverStationEnhancedIO *ControllBox; //ControllBox = switch box
    Jaguar *shooter1;  // 1 and 2 are right
    Jaguar *shooter2;  // 3 and 4 are left
    Jaguar *shooter3;
    Jaguar *shooter4;
    //DigitalInput *shooterDin;
    DigitalInput *Upperlimit;
    DigitalInput *Lowerlimit;
    Counter *shootercontador;
    Task *shooterspeedTask;
    Task *kickerTask;
    volatile bool Shooter_onoff;
    volatile bool kicker_onoff; 
    volatile bool kicker_in_motion;
    volatile double distanceInInches;
    Jaguar *BridgeBootMotor10; // motor 10
    volatile bool kicker_cancel;
    volatile bool kicker_down;
    
public:
	RobotDemo(void)		
	{
		kicker_in_motion = false;
		sol = new Solenoid(2);
		rightstick = new Joystick(1);
		leftstick = new Joystick(2);
		lonelystick = new Joystick (3);
		Motor1=new Jaguar(1);
		Motor2=new Jaguar(2);
		Motor3=new Jaguar(3);
		Motor4=new Jaguar(4);
		BallGathererMotor9 = new Jaguar(9);
		myRobot=new RobotDrive(Motor1,Motor2,Motor4,Motor3);
		rlyLED=new Relay(8,Relay::kForwardOnly);
		cam = &AxisCamera::GetInstance("10.8.12.11");
		cam->WriteResolution(AxisCameraParams::kResolution_160x120);
		myRobot->SetExpiration(0.5);
		ControllBox = & DriverStation::GetInstance()->GetEnhancedIO();
		shooter1 = new Jaguar(5); //front left
		shooter2 = new Jaguar(6);
		shooter3 = new Jaguar(7);
		shooter4 = new Jaguar(8);
		//shooterDin = new DigitalInput(1);
		shootercontador = new Counter(1);
		shootercontador->Start();
		shooterspeedTask = new Task("ShooterSpeed",(FUNCPTR)&shooterspeedloop);
		kickerTask = new Task ("Kicker", (FUNCPTR)&kickerloop);
		Upperlimit = new DigitalInput(3);
		Lowerlimit = new DigitalInput(2);
		kickermotor = new Relay (6, Relay::kBothDirections);
		BridgeBootMotor10 = new Jaguar(10);
		kicker_cancel = false;
		kicker_down = false;
		
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		myRobot->SetSafetyEnabled(false);
		
		//shooter on
		
		kickerTask->Start((UINT32)this);
		shooterspeedTask->Start((UINT32)this);
		
		Shooter_onoff=true;
				//if (speederror < 10);
		
		//track+adjust
		
		LEDLights(true);
		//turn tracking on
		//while (tracking(false) == false) {
	//	}
		//if while returns true, then shoot
		//might have to wait for encoder once capabilities have been enabled
		
		Wait(2.0);
  		AutonomousShooting(true);
  		AutonomousShooting(true);
  		AutonomousShooting(true);
 
		
		//load
/*
		Wait(1.0);
		kicker_onoff = true;
		while(kicker_in_motion == false) {
			Wait(0.005);
		printf ("kicker_onoff is false, kicker_onoff is true\n");
		}
		
		kicker_onoff = false;
		while (kicker_in_motion == true) {
			Wait(0.005);
		printf ("kicker_in_motion is true, kicker_onoff is false\n");
		}		
		
		//shoot, by itself, because the shooter motor was already on. 
		
		//gather
		
		ballgatherer(true, false);
		Wait(4.0);
		printf ("ballgatherer on\n");
		ballgatherer(false, false);
		printf ("ballgatherer off\n");
		
		//load
		
		kicker_onoff = true;
			while(kicker_in_motion == false) {
				Wait(0.005);
			printf ("kicker_in_motion is false, kicker_onoff is true\n");
			}
		
		kicker_onoff = false;
			while (kicker_in_motion == true) {
				Wait(0.005);
			printf ("kicker_in_motion is true, kicker_onoff is false \n");
			}	
		
			//shoot *does by itself because the shooter is already on, AGAIN! :D 
			
			ballgatherer(true, false);
			Wait(4.0);
			printf ("ballgatherer on\n");
			ballgatherer(false, false);
			printf ("ballgatherer off\n");
		
			kicker_onoff = true;
			while(kicker_in_motion == false) {
				Wait(0.005);
			printf ("kicker_onoff is false, kicker_onoff is true\n");
			}
			
			kicker_onoff = false;
			while (kicker_in_motion == true) {
				Wait(0.005);
			printf ("kicker_in_motion is true, kicker_onoff is false\n");
			}		
			
			//shoot, by itself, because the shooter motor was already on. 
			
			//gather
		
			
	*/
	
		kickerTask->Stop();
		shooterspeedTask->Stop();
		Shooter_onoff=false;
		
		LEDLights(false);
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	
	
	void OperatorControl(void)
	{
		myRobot->SetSafetyEnabled(false);
		
		LEDLights (true); //turn camera lights on
		
		shooterspeedTask->Start((UINT32)this); //start counting shooter speed
		
		kickerTask->Start((UINT32)this); //turns on the kicker task
		
		kicker_in_motion = false;
				
		while (IsOperatorControl() && !IsDisabled())
		{
			
			if (ControllBox->GetDigital(3)) //turn tracking on with switch 3 on controll box
			{ 
				tracking(ControllBox->GetDigital(7));
			}
			else 
			{
				myRobot->TankDrive(leftstick, rightstick); //if tracking is off, enable human drivers
				Wait(0.005);	// wait for a motor update time
			}

			Shooter_onoff=ControllBox->GetDigital(4); //shoot if switch 4 is on
		
			ballgatherer(ControllBox->GetDigital(5), rightstick->GetRawButton(10));
			 
			kicker_onoff=lonelystick->GetRawButton(1);
			
			bridgeboot(ControllBox->GetDigital(6));
			
			kicker_cancel=lonelystick->GetRawButton(2);
			
			//kicker_down=rightstick->GetRawButton(11));
			
		}
		
		
		LEDLights (false);
		shooterspeedTask->Stop();
		kickerTask->Stop();
		ballgatherer(false, false);
		kickermotor->Set(Relay::kOff);
	}
	
	void AutonomousShooting (bool onoff)
	{
		kicker_onoff = true;
		while(kicker_in_motion == false) {
			Wait(0.005);
		printf ("kicker_onoff is false, kicker_onoff is true\n");
		}
		
		kicker_onoff = false;
		while (kicker_in_motion == true) {
			Wait(0.005);
		printf ("kicker_in_motion is true, kicker_onoff is false\n");
		}		
		
		//shoot, by itself, because the shooter motor was already on. 
		
		//gather
		
		ballgatherer(true, false);
		Wait(4.0);
		printf ("ballgatherer on\n");
		ballgatherer(false, false);
		printf ("ballgatherer off\n");
	}
	
	void ShooterControl (bool onoff,double power) //function for the shooter
	{
		if (onoff = true)
		{
			shooter1->SetSpeed(-power);
			shooter2->SetSpeed(-power);
			shooter3->SetSpeed(power);
			shooter4->SetSpeed(power);
		}
		else
		{
			shooter1->SetSpeed(0.0);
			shooter2->SetSpeed(0.0);
			shooter3->SetSpeed(0.0);
			shooter4->SetSpeed(0.0);
		}

	}
	
	void LEDLights (bool onoff) //light function
	{
		if (onoff) {
			rlyLED->Set(Relay::kForward);// turn on LEDs
			
		} else {
			rlyLED->Set(Relay::kOff);// turn off LEDs
			
		}
	}
	
	double trackingFeedbackFunction(double x) {
		// takes a pixel distance x (horizontal # of pixels from center of target)
		// returns a speed [-1.0, +1.0] positive for right turn
		
		double deadZone = 4.0; // pixels on either side of x=0
		double speedslope = .003; // the slope in (speed/pixel)
		double minspeed = .45; // the minimun speed before reaching deadzone
		double maxspeed = .75; // the maximum speed robot can reach when turning
		
		if(x > -deadZone && x < deadZone)
		{
			// close enough to center -- stop turning
			return 0;
		}
		
		double speed = 0;
		
		if(x > 0) 
		{
			speed = speedslope*(x-deadZone) + minspeed; // slope of positive/right
		} 
		else 
		{
			speed = speedslope*(x+deadZone) - minspeed; //slope of negative/left
		}
		
		if(speed > maxspeed)
		{
			speed = maxspeed;
		}
		
		if(speed < -maxspeed)
		{
			speed = -maxspeed;
		}
		
		if(speed < 0) 
		{
			speed = speed;
		}
		
		return speed;
	}
	
	static void shooterspeedloop (RobotDemo *robot) //counting shooter speed
	{
		int store;
		double counterspeed; // rpm
		double wantedspeed;
		//double maxrpm = 2600;
		double speederror;
		double pulsesPerRevolution = 6;
		double waitingTime = 0.25; // seconds
		double power = 1.0;
		
		while(true) {
			
			// DO STUFF 
			// reset counter
			robot->shootercontador->Reset();
			
			// start counter
			
			// wait ?? sec
			Wait(waitingTime);
			
			// take reading
			if (robot->IsAutonomous()){ 
			  	  //wantedspeed = robot->rpmcalculation(robot->distanceInInches);
				wantedspeed = 3400.0;
			  }
			  if (robot->IsOperatorControl()   ){
			  	/*  if ( (robot->ControllBox->GetDigital(1)) && (robot->ControllBox->GetDigital(2) == false)  ){//1 on and 2 off
			  	  	 wantedspeed = robot->rpmcalculation(robot->distanceInInches);
			  	  	printf("auto_rpm_determination\n");
			  	  }
			  	  */
				  if ( (robot->ControllBox->GetDigital(1)) && (robot->ControllBox->GetDigital(2) == false)) {
					  wantedspeed = (  850.0*( -robot->lonelystick->GetZ() + (1.0) ) + 1700.0 );
					  printf("manual speed %f\n", wantedspeed);
				  }
			  	  else if ( (robot->ControllBox->GetDigital(1) == false) && (robot->ControllBox->GetDigital(2) == false)){ //1 off and 2 off
			  	  	wantedspeed = 3400.0;
			  	  printf("maxspeed\n");
			  	  }
			  	  else if ( (robot->ControllBox->GetDigital(1)) && (robot->ControllBox->GetDigital(2)) ){ //1 on and 2 on
			  	   	wantedspeed = 2700.0;
			  	  printf("minspeed\n");
			  	  }
			  	  
			  }
			 
			
			//wantedspeed = robot->rpmcalculation(robot->distanceInInches);
			store = robot->shootercontador->Get();
			
			counterspeed = (store / pulsesPerRevolution)*(60.0/waitingTime); //revolutions in half a second * 120 for minute			
			speederror = wantedspeed - counterspeed;
			printf ("Shooter Speed: %f RPM (%d pulses)\n", counterspeed, store);
			printf ("Shooter Speed Error: %f RPM\n", speederror);
			//calculate power, below
			power = power + speederror*(1.0/20000.0);
			//proportional = distance you want it to travel (p = actual distance - wanted distance)
			
			if(power > 1.0){
				power=1.0;
			}
			
			
			//else
			if(robot->Shooter_onoff == false){
				power = 0.0;

			}
			printf ("Shooter power set to: %f\n", power);
			robot->ShooterControl (robot->Shooter_onoff,power);
		}

	}
	
	double rpmcalculation (double distanceInInches)
	{
		double rpm;
		rpm = 22.0*(distanceInInches-144.0) + 3200.0;
		printf("%f rpm\n",rpm);
		printf("%f distance in inches,mm \n",distanceInInches);

		return rpm;

		
		
	}
	
	void bridgeboot (bool onoff)
	{
		if (onoff == true) 
		{
			BridgeBootMotor10->SetSpeed(-1.0);
		}
		
		else
		{
			BridgeBootMotor10->SetSpeed(0.5);
			}
	}
	
	void ballgatherer (bool onoff, bool reverse)
	{
		if (kicker_in_motion == true){
			BallGathererMotor9->SetSpeed(0.0);
			return;
		} 
		if (onoff == true){
			if (reverse == true) {
				BallGathererMotor9->SetSpeed(0.8);
				}
			else {
				BallGathererMotor9->SetSpeed(-1.0);
				}
		}
		else{
			BallGathererMotor9->SetSpeed(0.0);
		}
		
		//printf ("%d %f ballgatherer onoff:\n");
	}
	
	static void kickerloop (RobotDemo *robot)
	{
		while (true){
			if(robot->kicker_onoff == false) {
				// if the button is *not* pressed, then wait and check again
				Wait(0.005);
				continue;
			}
			
			printf("kickerloop: starting forward\n");
			
			// if we got here in the loop, then kicker_onoff == true
			robot->kicker_in_motion = true;
			robot->kickermotor->Set(Relay::kForward); //turn kicker motor on
			
			while (robot->Upperlimit->Get() && !robot->kicker_cancel) {
				printf("Hit upperlimit\n");
				// while upper limit not hit yet
				Wait(0.001);
			}
			if (robot->kicker_cancel == true) {
				robot->kicker_cancel = false;
				robot->kickermotor->Set(Relay::kOff);
				robot->kicker_in_motion = false;
				printf("kicker cancelled\n");

				continue;
			}
				
/*
 * 			if (robot->kicker_down == true) {
 * 			robot->kicker_cancel = false;
 * 			robot->kickermotor->Set(Relay::kReverse);
 * 			robot->kicker_in_motion = false;
 * 			printf("kicker manual reverse\");
 * 			
 * 			contiue;
 * 			
 * 			}
 */
			printf("kickerloop: hit upper limit.  off & waiting.\n");
			robot->kickermotor->Set(Relay::kOff);
			Wait(1.0);
			robot->kickermotor->Set(Relay::kReverse);
			printf("kickerloop: hit upper limit.  reverse.\n");
			
			while (robot->Lowerlimit->Get() && !robot->kicker_cancel) {
				// while lower limit not hit yet
				Wait(0.001);

			}
			if (robot->kicker_cancel == true) {
				robot->kicker_cancel = false;
				robot->kickermotor->Set(Relay::kOff);
				robot->kicker_in_motion = false;
				printf("kicker cancelled\n");

				continue;
			}
			printf("kickerloop: lower limit hit.  off.\n");

			robot->kicker_in_motion = false;
			robot->kickermotor->Set(Relay::kOff);								
			
		}
		
		
	}
	
	bool tracking (bool use_alternate_score) //camera tracking function
	{
		//takes one camera frame and turns towards tallest target
		//returns true if target is within deadzone, returns false otherwise
		Threshold tapeThreshold(0, 255, 0, 90, 220, 255); //red hsl as of 20110303, this is the hue, saturation and luminosicity ranges that we want
		BinaryImage *tapePixels;//
		Image *convexHull;
		BinaryImage *convexHullBinaryImage;
		ParticleAnalysisReport par;//analyzed blob (pre convex hull)
		ParticleAnalysisReport convexpar;// ONE filled-in blob
		vector<ParticleAnalysisReport>* pars;//where many analyzed blob goes (pre)
		vector<ParticleAnalysisReport>* convexpars; //where MANY  filled-in blobs go
		
		bool foundAnything = false;	
		double best_score = 120;
		double best_speed;
		double particle_score;
		
		ImageType t;
		int bs;
		img = cam->GetImage(); 
		printf("cam->GetImage() returned frame %d x %d\n",img->GetWidth(),img->GetHeight());
		tapePixels = img->ThresholdHSL(tapeThreshold);
		imaqGetBorderSize(tapePixels->GetImaqImage(),&bs);
		imaqGetImageType(tapePixels->GetImaqImage(),&t);
		convexHull = imaqCreateImage(t,bs);
		convexHullBinaryImage = new BinaryImageWrapper(convexHull);
		convexHullBinaryImage->GetOrderedParticleAnalysisReports();
		//tapePixels = img->ThresholdHSL(int 0,int 50,int -100,int -50,int luminenceLow,int luminanceHigh);
		pars = tapePixels->GetOrderedParticleAnalysisReports();
		imaqConvexHull(convexHull,tapePixels->GetImaqImage(),true);
		convexHullBinaryImage = new BinaryImageWrapper(convexHull);
		convexpars = convexHullBinaryImage->GetOrderedParticleAnalysisReports();
		//imaqGetParticleInfo()
		
		//convexpars = convexHull->GetOrderedParticleAnalysisReports();
		for (int i=0;i < convexHullBinaryImage->GetNumberParticles();i++)
		{
			//par = (*pars)[0];
			//convexpar = (*convexpars)[i];
			convexpar = convexHullBinaryImage->GetParticleAnalysisReport(i);
			par = tapePixels->GetParticleAnalysisReport(i);
			

			if((convexpar.boundingRect.width < 10) || (convexpar.boundingRect.height < 7))
			{									
				continue;
		    }
//				printf("%d  par:%f convex:%f particle area\n",i,par.particleArea,convexpar.particleArea);
			
			if ((par.particleArea/convexpar.particleArea > 0.4))
			{
				printf("%d skip max fillness ratio\n",i);
				continue;
			}
			if ((par.particleArea/convexpar.particleArea < 0.10))
			{
				printf("%d skip min fillness ratio\n",i);
				continue;
			}
			
			if((double)(convexpar.boundingRect.width)/(double)(convexpar.boundingRect.height)>1.8)
			{
				printf("%d skip max aspect ratio\n",i);
				continue;
			}
			
			if((double)(convexpar.boundingRect.width)/(double)(convexpar.boundingRect.height)<.8)
			{
				printf("%d skip min aspect ratio\n",i);
				continue;
			}
			
			
						
			//printf("%f center of mass x\n",par.center_mass_x_normalized);
			//printf("%f center of mass y\n",par.center_mass_y_normalized);
			distanceInInches = (18.0*179.3)/(convexpar.boundingRect.height);
			double pwidth = convexpar.boundingRect.width;
			double mwidth = ((double)convexpar.boundingRect.left+(double)convexpar.boundingRect.width*0.5);
			double angle = ((180.0/3.14159)*acos     (pwidth * distanceInInches/179.3/24.0)  );
			if(angle != angle) angle = 0.0; // if angle is NaN, set to zero
			printf("%f distance in inches\n",distanceInInches);
			//printf("%f angle\n",(180.0/3.14159)*acos     (pwidth * distanceInInches/415.0/24.0)  );
			printf("%d BBctrX:%f CMX:%f\n", i, (double)convexpar.boundingRect.left + (double)convexpar.boundingRect.width*0.5, (double)par.center_mass_x);		
			//printf("%f angle2\n",(((pwidth * distanceInInches)/415.0)/24.0));
			//printf("%f center of mass x\n",par.center_mass_x_normalized);
			printf("%d %f %f center of mass x\n",i,convexpar.center_mass_x_normalized,par.center_mass_x_normalized);
			printf("%d %f %f center of mass y\n",i,convexpar.center_mass_y_normalized,par.center_mass_y_normalized);
			printf("%d %f %f rectangle score\n",i,(convexpar.particleArea)/((convexpar.boundingRect.width)*(convexpar.boundingRect.height))*(100),(par.particleArea)/((par.boundingRect.width)*(par.boundingRect.height))*(100));
			printf("%d %f fillness ratio\n",i,par.particleArea/convexpar.particleArea);
			printf("%d %d %d width and height\n",i,(convexpar.boundingRect.width),(convexpar.boundingRect.height));
			printf("%d %f aspect ratio\n",i,((convexpar.boundingRect.width)/(double)(convexpar.boundingRect.height)));
			

			if ((double)(par.center_mass_x)>mwidth)
				{
				 angle=angle*(-1.0);
				}
			 printf("%f true angle\n",angle);
			 //Wait(1.0);
			 
			 double aiming_target_offset = 0.0; 
			 //aiming_target_offset = pwidth * angle * (-0.5 / 45.0); numbers are iffy -> NaN
			 
			 
			 double speed = trackingFeedbackFunction(mwidth + aiming_target_offset - 80.0);
			 printf("%f aiming_target_offset due to %f degree angle\n", aiming_target_offset, angle);
			 printf("%f x offset \n",mwidth + aiming_target_offset - 80.0);
			 printf("%f speed \n", speed);
			foundAnything = true;
			
			if (use_alternate_score == false){
				particle_score = convexpar.center_mass_y;
			}
			else{
				particle_score = 2.0*fabs((double)convexpar.center_mass_y - 60.0) + fabs((double)convexpar.center_mass_x - 80.0);
			}
			
			// keep track of the *lowest* score
			if (best_score > particle_score)
			{
				best_score = particle_score;
				best_speed = speed;
			}
		}
		
		if(foundAnything == false) {
			myRobot->TankDrive(0.0, 0.0);	
		}
		else
		{
			myRobot->TankDrive(-best_speed,best_speed);
		}
		
		 
	    delete img;
		delete tapePixels;
		delete pars;				
		delete convexHullBinaryImage;
		delete convexpars;
		//imaqDispose(convexHull);
		
		if (foundAnything && best_speed == 0.0){
			return true;
		}
		
		else
		{
			return false;
		}
	}
};
START_ROBOT_CLASS(RobotDemo);

/*
 * need to:
 * change autonomous gather/kick/shoot thing into a function for more readability
 * use angle to measure the spot that we want to shoot at if the robot is at an angle
 * fine tune the distance calculation
 * use distance to use auto speed to use in autonomous
 */
