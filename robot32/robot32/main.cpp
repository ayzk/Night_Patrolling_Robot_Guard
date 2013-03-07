
#include "Aria.h"
#include <WINSOCK2.H>
#include <stdio.h>
#include<math.h>
#include<time.h>
#pragma comment(lib,"ws2_32.lib")
#define pi 3.14159265
#define TurnDelta 10
#define port 11111
#define LarX 2500
#define LarY 1500

// using namespace std;
	ArRobot *Myrobot;
	void setRobot(ArRobot * robot){
		Myrobot=robot;
	}
	time_t newtime,oldtime;
	SOCKET serConn[3];
void send(int px,int py,int x,int y)
{
			char xp[100];
			char yp[100];
			printf("%d,%d,%d,%d\n",px,py,x,y);

			itoa(px,xp,10);
			itoa(py,yp,10);
			strcat(xp,",");
			strcat(xp,yp);
			strcat(xp,",");
			//x y
			itoa(x,yp,10);
			strcat(xp,yp);
			strcat(xp,",");
			itoa(y,yp,10);
			strcat(xp,yp);
			printf("%s",xp);
			send(serConn[2],(char*)xp,strlen(xp),0);
}
int main(int argc, char **argv)
{


	//net
	WORD myVersionRequest;
	WSADATA wsaData;
	myVersionRequest=MAKEWORD(1,1);
	int err;
	err=WSAStartup(myVersionRequest,&wsaData);
	if (err)
	{
		printf("error\n");

	}
	SOCKET serSocket=socket(AF_INET,SOCK_STREAM,0);
	SOCKADDR_IN addr;
	addr.sin_family=AF_INET;
	addr.sin_addr.S_un.S_addr=htonl(INADDR_ANY);
	addr.sin_port=htons(port);
 
	bind(serSocket,(SOCKADDR*)&addr,sizeof(SOCKADDR));
	listen(serSocket,5);



	//robot
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArSimpleConnector simpleConnector(&parser);
	ArRobot robot;
	setRobot(&robot);
	ArSonarDevice sonar;
	ArAnalogGyro gyro(&robot);
	robot.addRangeDevice(&sonar);

	// Make a key handler, so that escape will shut down the program
	// cleanly
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	printf("You may press escape to exit\n");

	// Collision avoidance actions at higher priority
	ArActionLimiterForwards limiterAction("speed limiter near",100, 600, 250);
	ArActionLimiterForwards limiterFarAction("speed limiter far", 300, 1100, 400);
	ArActionLimiterTableSensor tableLimiterAction;
	robot.addAction(&tableLimiterAction, 100);
	robot.addAction(&limiterAction, 95);
	robot.addAction(&limiterFarAction, 90);

	// Goto action at lower priority
	ArActionGoto gotoPoseAction("goto");
	robot.addAction(&gotoPoseAction, 50);
  
	// Stop action at lower priority, so the robot stops if it has no goal
	ArActionStop stopAction("stop");
	robot.addAction(&stopAction, 40);

	// Parse all command line arguments
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{    
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}
  
	// Connect to the robot
	if (!simpleConnector.connectRobot(&robot))
	{
		printf("Could not connect to robot... exiting\n");
		Aria::exit(1);
		return 1;
	}
	robot.runAsync(true);

	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG, 0);


	bool first = true;
	int goalNum = 0;
	ArTime start;
	start.setToNow();
	//net
	SOCKADDR_IN Ksocket,Wsocket,Csocket;
	int len=sizeof(SOCKADDR);
	char recvbuf[1];
	
	//kinect
	serConn[0]=accept(serSocket,(SOCKADDR*)&Ksocket,&len);
	
	printf("kinect  %s  connected\n",inet_ntoa(Ksocket.sin_addr));
	//weapon
	serConn[1]=accept(serSocket,(SOCKADDR*)&Wsocket,&len);

	printf("Weapon %s  connected",inet_ntoa(Wsocket.sin_addr));

	//Client
	serConn[2]=accept(serSocket,(SOCKADDR*)&Csocket,&len);

	printf("Client %s  connected\n",inet_ntoa(Csocket.sin_addr));
	char Krecvbuf[8];
	char Csendbuf[16];
	char Crecvbuf[255];
	char xp[10];
	char yp[10];
	int px,py;
	int ReceiveTimeout = 0; 
	// If iMode!=0, non-blocking mode is enabled.
	u_long iMode=1;
	ioctlsocket(serConn[0],FIONBIO,&iMode);

	int x=0,y=0;
	bool rflag=false;
	oldtime=time(NULL);
	bool tracing=0;
	while (1) 
	{	
		int bnum=recv(serConn[0],Krecvbuf,8,0);
		//printf("recv %d byte\n",bnum);
		if (bnum>0){
			
			memcpy(&x,Krecvbuf,4);
			memcpy(&y,Krecvbuf+4,4);
			if(x==-1&&y==-1)
			{
				printf("%d %d \n",x,y);
				if(rflag)
				{
					rflag=false;
					printf("miss");
					robot.clearDirectMotion();
					first=true;
				}
			}else 
			{
				if (rflag==false){//gotoPoseAction.cancelGoal();
				printf("cancel goal");
				send(serConn[1],"1",strlen("1"),0);
				rflag=true;}
			}
			//printf("%d,%d\n",Krecvbuf[0],Krecvbuf[4]);
			send(robot.getX(),robot.getY(),x,y);
			
		}
		
		printf( "1 Going to next goal at %.0f %.0f\n", 
				gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY());
		printf( "first is %d",first);
		//bum=recv(serConn[0],Crecvbuf,)
		robot.lock();
		
		// Choose a new goal if this is the first loop iteration, or if we 
		// achieved the previous goal.
		if(rflag)
		{
			
			double delta=atan2(double(x),double(y))/pi*180;
			printf("%d %d\n"
				,x,y);
			if(delta>TurnDelta||delta<-TurnDelta)robot.setDeltaHeading(delta);
			robot.move(y);
			//Sleep(1000);
		}
		else 
		if (first || gotoPoseAction.haveAchievedGoal())
		{
			first = false;
			goalNum++;
			if (goalNum > 4){
				goalNum=1;
			}// start again at goal #1
				//gotoPoseAction.cancelGoal();
			double px,py;
			// set our positions for the different goals
			if (goalNum == 1)
			gotoPoseAction.setGoal(ArPose(LarX, 0));
			else if (goalNum == 2)
			gotoPoseAction.setGoal(ArPose(LarX, LarY));
			else if (goalNum == 3)
			gotoPoseAction.setGoal(ArPose(0, LarY));
			else if (goalNum == 4)
			gotoPoseAction.setGoal(ArPose(0, 0));

			printf( "2 Going to next goal at %.0f %.0f\n", 
				gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY());
		}

		/*if(start.mSecSince() >= duration) {
			printf( "%d seconds have elapsed. Cancelling current goal, waiting 3 seconds, and exiting.", duration/1000);
			gotoPoseAction.cancelGoal();
			robot.unlock();
			ArUtil::sleep(3000);
			break;
		}*/

		robot.unlock();
		ArUtil::sleep(100);
	}
  
	// Robot disconnected or time elapsed, shut down
	Aria::exit(0);
	return 0;
}
