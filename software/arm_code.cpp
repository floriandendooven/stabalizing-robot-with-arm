#include <assert.h>


#include <unistd.h>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <iostream>
extern "C" {
#include <wiringPi.h>
}

using namespace std;

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"




#define N_SLAVES_CONTROLED 6
#define PIN_RIGHT 25
#define PIN_LEFT 24

#define restposition 1
// case 1
#define angle_case1 39.1
#define step_case1 3
#define side 1 //left 1; right 2

// case 2
#define duration_case2 10000

// case 3
#define angle_case3 150
#define step_case3 5
// case 4
#define step_case4 5
#define startfalling_case4 40
#define maxangle_case4 65
#define kd_case4 0.3
#define duration_case4 6000


//case 5
#define step_case5 4
#define step_case5_1 1
#define up_case5 0.12
#define kp_case5 0.5
#define speed_case5 0.9
//case 6
#define step_case6 5
//case 7
#define step_case7 5
//case 9
#define step_case9 5
//case11
#define K_case11 0.03
#define K_case11_1 0.1
#define c 0.5
#define duration_case11 200000
//case 12
#define step_case12 5
//case 13
#define step_case13 5

int main(int argc, char **argv)
{

// Wiring setup
wiringPiSetup();
pinMode(25, OUTPUT);
pinMode(24, OUTPUT);
digitalWrite(25,0);
digitalWrite(24,0);



    if(argc != 2)
    {
        throw std::runtime_error("Please provide the interface name "
                                 "(i.e. using 'ifconfig' on linux");
    }

	int cpt = 0;
	double dt = 0.001;
	double t1;
	double t3;
	double t4;
	double t5;
	double t6;
	double t7;
	double t9;
	double t11;
	double t12;
	double t13;
	double kp = 3;
	double kd = 0.1;
	double iq_sat = 10.0;
    double init_pos[N_SLAVES * 2] = {0};
	int state =0;
	double pi = 3.1415;
	bool up;
	//case1
    double lastpos1_case1;
	
    //case2
    int i_case2;
    // case3
	double lastpos0_case3;	
	//case 4;
	double check_case4;
    double check_case4_1=9999;
	double lastpos1_case4;
	//case 5;
	double lastcurrent_case5;
	double initpos1_vs;
	double initpos0_vs;
	double uppos_case5;
	double lastpos0_case5;
	//case 7
	double lastpos1_case7;
	//case8
	int i_case8;
    //case 11
    double lastpos1_case11;
	//case 12
    double lastpos0_case12;




	nice(-20); //give the process a high priority
	printf("-- Main --\n");
	MasterBoardInterface robot_if(argv[1]);
	robot_if.Init();
	//Initialisation, send the init commands
	for (int i = 0; i < N_SLAVES_CONTROLED; i++)
	{
		robot_if.motor_drivers[i].motor1->SetCurrentReference(0);
		robot_if.motor_drivers[i].motor2->SetCurrentReference(0);
		robot_if.motor_drivers[i].motor1->Enable();
		robot_if.motor_drivers[i].motor2->Enable();
		
		// Set the gains for the PD controller running on the cards.
		robot_if.motor_drivers[i].motor1->set_kp(kp);
		robot_if.motor_drivers[i].motor2->set_kp(kp);
		robot_if.motor_drivers[i].motor1->set_kd(kd);
		robot_if.motor_drivers[i].motor2->set_kd(kd);

		// Set the maximum current controlled by the card.
		robot_if.motor_drivers[i].motor1->set_current_sat(iq_sat);
		robot_if.motor_drivers[i].motor2->set_current_sat(iq_sat);
		
		robot_if.motor_drivers[i].EnablePositionRolloverError();
		robot_if.motor_drivers[i].SetTimeout(5);
		robot_if.motor_drivers[i].Enable();
	}

	std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
	while (!robot_if.IsTimeout() && !robot_if.IsAckMsgReceived()) {
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
		{
			last = std::chrono::system_clock::now();
			robot_if.SendInit();
		}
	}

	if (robot_if.IsTimeout())
	{
		printf("Timeout while waiting for ack.\n");
	}

	while (!robot_if.IsTimeout())
	{
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > dt)
		{
			last = std::chrono::system_clock::now(); //last+dt would be better
			cpt++;
			robot_if.ParseSensorData(); // This will read the last incomming packet and update all sensor fields.
			switch (state)
			{
			case 0: //check the end of calibration (are the all controlled motor enabled and ready?)
				//state =restposition;
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					state=restposition;
					if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

					if (!(robot_if.motors[i].IsEnabled() && robot_if.motors[i].IsReady()))
					{
						state = 0;
					}
					init_pos[i] = robot_if.motors[i].GetPosition(); //initial position

					// Use the current state as target for the PD controller.
					robot_if.motors[i].SetCurrentReference(0.);
					robot_if.motors[i].SetPositionReference(init_pos[i]);
					robot_if.motors[i].SetVelocityReference(0.);
					
					t1 = 0;	
					t3 = 0;
					t4 = 0;
					t5 = 0;
					t6 = 0;
					t7 = 0;
					t9 = 0;
					t11 = 0;
					t12 = 0;
					t13 = 0;
       				//case2
      				i_case2=0;
					// case8
					i_case8=0;
					up=false;
                                         if(i==1){
                                                 robot_if.motor_drivers[i / 2].motor2->set_kp(kp);
                                                 robot_if.motor_drivers[i / 2].motor2->set_kd(kd);
                                         }


					}
					break;

			case 1:  // arm goes up from starting position 
				//closed loop, position
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (i % 2 == 0)
					{
						if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

						// making sure that the transaction with the corresponding µdriver board succeeded
						if (robot_if.motor_drivers[i / 2].error_code == 0xf)
						{
							//printf("Transaction with SPI%d failed\n", i / 2);
							continue; //user should decide what to do in that case, here we ignore that motor
						}
					}

					if (robot_if.motors[i].IsEnabled())
					{
							state = 1;
						// i=0 is right motor on driver board, i=1 is left motor in driver board!
						if(i==0){
							robot_if.motors[i].SetPositionReference(init_pos[i]);
						}

						if(i==1){
							double ref1=init_pos[i]-step_case1*t1;
							robot_if.motors[i].SetPositionReference(ref1);
							if(robot_if.motors[i].GetPosition() <init_pos[i]-angle_case1*9/180*pi){
								lastpos1_case1=robot_if.motors[i].GetPosition();
								state = 2;
							}
						}
					}
				}
				t1+=dt;
				break;

			case 2:  //arm turns to the side of the railing
				//closed loop, position
				for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
				{
					if (i % 2 == 0)
					{
						if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave
 
						// making sure that the transaction with the corresponding µdriver board succeeded
						if (robot_if.motor_drivers[i / 2].error_code == 0xf)
						{
							//printf("Transaction with SPI%d failed\n", i / 2);
							continue; //user should decide what to do in that case, here we ignore that motor
						}
					}
						// i=0 is rechts, i=1 is links!!!
						if(i==0){
	                                                robot_if.motors[i].SetPositionReference(init_pos[i]);
						}
						if(i==1){
							robot_if.motors[i].SetPositionReference(lastpos1_case1);
						}
						
						if(side==1){
							digitalWrite(25,1);
							digitalWrite(24,0);
						}else{
							digitalWrite(25,0);
							digitalWrite(24,1);
						}
						
						if(i_case2==duration_case2){
							state=3;
						}
					i_case2+=1;

				}

			break;

                        case 3: //arm extend  starting from init position 
                                 //closed loop, position
                                 for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                 {
                                         if (i % 2 == 0)
                                         {
                                                 if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected sla
                                                 // making sure that the transaction with the corresponding µdriver board succeeded
                                                 if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                 {
                                                         //printf("Transaction with SPI%d failed\n", i / 2);
                                                         continue; //user should decide what to do in that case, here we ignore that motor
                                                 }
                                         }

                                         if (robot_if.motors[i].IsEnabled())
                                         {
                                                 if(i==1){
                                                         robot_if.motors[i].SetPositionReference(lastpos1_case1);
					                }

                                                 if(i==0){
                                                         double ref=init_pos[i]+step_case3*t3;
                                                         robot_if.motors[i].SetPositionReference(ref);
                                                       	 if(robot_if.motors[i].GetPosition()>init_pos[i]+angle_case3*9/180*pi){
                                                                lastpos0_case3=robot_if.motors[i].GetPosition();  
                                                              state =4;
                                                        }
                                                 }
                                         }
                                 }
				t3+=dt;
                                 break;




                        case 4: //arm go down and check if there is a railing
                                 //closed loop, position
                                 for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                 {
                                         if (i % 2 == 0)
                                         {
                                                 if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected sla
                                                 // making sure that the transaction with the corresponding µdriver board succeeded
                                                 if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                 {
                                                         //printf("Transaction with SPI%d failed\n", i / 2);
                                                         continue; //user should decide what to do in that case, here we ignore that motor
                                                 }
                                         }

                                         if (robot_if.motors[i].IsEnabled())
                                         {
                                                 if(i==1){
							                        if(t4<startfalling_case4*dt){ //the arm goes down with help of the motor, this is done to be sure the arm starts falling
        	                                            robot_if.motor_drivers[i / 2].motor2->set_kp(kp);
	                                                    robot_if.motor_drivers[i / 2].motor2->set_kd(kd);
								                        double ref=lastpos1_case1+step_case4*t4;
	                                                    robot_if.motors[i].SetPositionReference(ref);
								                        check_case4=robot_if.motors[i].GetPosition();
							                        }else{ //the arm makes a damped fall
                                                        robot_if.motor_drivers[i / 2].motor2->set_kp(0);
                                                        robot_if.motor_drivers[i / 2].motor2->set_kd(kd_case4);
							                            if(t4>duration_case4*dt ||	check_case4_1== check_case4 ==robot_if.motors[i].GetPosition()){ //railing found
								                        state=5;
								                        lastpos1_case4=robot_if.motors[i].GetPosition();
							                            }
							                            if(robot_if.motors[i].GetPosition()>lastpos1_case1+maxangle_case4*9/180*pi){  //no railing found
                                                            lastpos1_case4=robot_if.motors[i].GetPosition();
                                                            lastpos1_case7=lastpos1_case4;
                                                            state=6;
							                                robot_if.motor_drivers[i / 2].motor2->set_kp(kp);
                                                            robot_if.motor_drivers[i / 2].motor2->set_kd(kd);
							                                }
							                            }       
						                        lastpos1_case4=robot_if.motors[i].GetPosition();
                                                check_case4_1=check_case4;
						                        check_case4=robot_if.motors[i].GetPosition();
						                        }
                                                if(i==0){
                                                    robot_if.motors[i].SetPositionReference(lastpos0_case3);
                                                }
                	                   }
                                 }
				t4+=dt;
                                 break;
                                 
			case 5: //arm retract to search for the railing

                                 //closed loop, position
                                 for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                 {
                                         if (i % 2 == 0)
                                         {
                                                 if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected sla
                                                 // making sure that the transaction with the corresponding µdriver board succeeded
                                                 if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                 {
                                                         //printf("Transaction with SPI%d failed\n", i / 2);
                                                         continue; //user should decide what to do in that case, here we ignore that motor
                                                 }
                                         }

                                         if (robot_if.motors[i].IsEnabled())
                                         {


					                        if(up==false){ // arm search fot the upwards movement of the endeffector
	                                                if(i==1){
                                                        robot_if.motor_drivers[i / 2].motor2->set_kp(0);
                                                	    robot_if.motor_drivers[i / 2].motor2->set_kd(0);
								                        if(robot_if.motors[i].GetPosition()<lastpos1_case4-up_case5){
									                    up=true;
									                    uppos_case5=robot_if.motors[i].GetPosition();
									                    lastpos0_case5=robot_if.motors[0].GetPosition();
								                        }
                                        	        }
                                	                if(i==0){ //arm retract
                        	                                double ref=lastpos0_case3-step_case5*t5;
                	                                        robot_if.motors[i].SetPositionReference(ref);                                                       		
								                            if(robot_if.motors[i].GetPosition()<lastpos0_case3-angle_case3*9/180*pi){			// no railing found
									                            state=7;
									                            lastpos1_case7=robot_if.motors[1].GetPosition();
								                            }
      							                    } 
				                                }else{
                                                    if(robot_if.motors[1].GetVelocity()>speed_case5){ //arm search for downwards movement of the endeffector
                                                        state=11;
                                                        initpos1_vs=robot_if.motors[1].GetPosition()+0.3; // init position motor 1 to start the virtual spring 
                                                        initpos0_vs=robot_if.motors[0].GetPosition()-0.05; // init position motor 0 to start the virtual spring
                                                        robot_if.motors[1].SetPositionReference(initpos1_vs);
                                                        robot_if.motors[0].SetPositionReference(initpos0_vs);	
						                                }	
							                        if(i==0){
								                        double ref=lastpos0_case5-step_case5_1*t5;
                                                        robot_if.motors[i].SetPositionReference(ref);
							                            if(robot_if.motors[i].GetPosition()<lastpos0_case3-angle_case3*9/180*pi){                       // no railing found
                                                                        state=7;
                                                                        lastpos1_case7=robot_if.motors[1].GetPosition();
                                                        }						
                                                    }
                                                    if(i==1){
                                                        robot_if.motor_drivers[1 / 2].motor2->set_kp(kp_case5);
                                                        robot_if.motor_drivers[1 / 2].motor2->set_kd(0);
                                                        robot_if.motors[i].SetPositionReference(lastpos1_case4+0.02);	
                                                    }
                                                    
						                        }
					                        }
                                 }
                                t5+=dt;
                                 break;

                        case 6: //arm  retract fully
                                 //closed loop, position
                                 for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                 {
                                         if (i % 2 == 0)
                                         {
                                                 if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected sla
                                                 // making sure that the transaction with the corresponding µdriver board succeeded
                                                 if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                 {
                                                         //printf("Transaction with SPI%d failed\n", i / 2);
                                                         continue; //user should decide what to do in that case, here we ignore that motor
                                                 }
                                         }

                                         if (robot_if.motors[i].IsEnabled())
                                         {
                                            if(i==1){
                              				    robot_if.motors[i].SetPositionReference(lastpos1_case4);                      
                                            }
                                            if(i==0){
                                                if(robot_if.motors[i].GetPosition()<init_pos[i]){
                                                    state=7;
                                                }
                                                double ref=lastpos0_case3-step_case6*t6;
                                                robot_if.motors[i].SetPositionReference(ref);
                                            }
                                         }
                                 }
                                t6+=dt;
                                 break;


                        case 7:  // arm goes up from last position 
                                //closed loop, position
                                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                {
                                        if (i % 2 == 0)
                                        {
                                                if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                                                // making sure that the transaction with the corresponding µdriver board succeeded
                                                if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                {
                                                        //printf("Transaction with SPI%d failed\n", i / 2);
                                                        continue; //user should decide what to do in that case, here we ignore that motor
                                                }
                                        }

                                        if (robot_if.motors[i].IsEnabled())
                                        {
                                                // i=0 is right motor on driver board, i=1 is left motor in driver board!
                                                if(i==0){
                                                        robot_if.motors[i].SetPositionReference(init_pos[i]);
                                                }

                                                if(i==1){
                                                        if(robot_if.motors[i].GetPosition()<lastpos1_case1){
                                                                state=8;
                                                        }
							                            robot_if.motor_drivers[i / 2].motor2->set_kp(kp);
                                                        robot_if.motor_drivers[i / 2].motor2->set_kd(kd);
                                                        double ref=lastpos1_case4-step_case7*t7;
                                                        robot_if.motors[i].SetPositionReference(ref);
                                                }
                                        }
                                }
                                t7+=dt;
                                break;




                        case 8:  //arm goes back to the middle 
                                //closed loop, position
                                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                {
                                        if (i % 2 == 0)
                                        {
                                                if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                                                // making sure that the transaction with the corresponding µdriver board succeeded
                                                if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                {
                                                        //printf("Transaction with SPI%d failed\n", i / 2);
                                                        continue; //user should decide what to do in that case, here we ignore that motor
                                                }
                                        }
                                                // i=0 is rechts, i=1 is links!!!
                                                if(i==0){
                                                        robot_if.motors[i].SetPositionReference(init_pos[i]);
                                                }
                                                if(i==1){
                                                        robot_if.motors[i].SetPositionReference(lastpos1_case1);
                                                }

                                               
                                               digitalWrite(25,0);
                                               digitalWrite(24,0);
                                                

                                                if(i_case8==duration_case2){
                                                        state=9;
                                                }
                                        i_case8+=1;

                                }

                        break;

                        case 9:  // arm goes back to its restposition
                                //closed loop, position
                                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                {
                                        if (i % 2 == 0)
                                        {
                                                if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                                                // making sure that the transaction with the corresponding µdriver board succeeded
                                                if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                {
                                                        //printf("Transaction with SPI%d failed\n", i / 2);
                                                        continue; //user should decide what to do in that case, here we ignore that motor
                                                }
                                        }

                                        if (robot_if.motors[i].IsEnabled())
                                        {
                                                // i=0 is right motor on driver board, i=1 is left motor in driver board!
                                                if(i==0){
                                                        robot_if.motors[i].SetPositionReference(init_pos[i]);
                                                }

                                                if(i==1){
                                                        if(robot_if.motors[i].GetPosition()>init_pos[i]){
                                                                state=10;
                                                        }
                                                        double ref=lastpos1_case1+step_case9*t9;
                                                        robot_if.motors[i].SetPositionReference(ref);
                                                }
                                        }
                                }
                                t9+=dt;
                                break;

		case 10:  //turn of all the motors
                                //closed loop, position
                                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                {
                                        if (i % 2 == 0)
                                        {
                                                if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                                                // making sure that the transaction with the corresponding µdriver board succeeded
                                                if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                {
                                                        //printf("Transaction with SPI%d failed\n", i / 2);
                                                        continue; //user should decide what to do in that case, here we ignore that motor
                                                }
                                        }

                                        if (robot_if.motors[i].IsEnabled())
                                        {
						
						                    if(i==0){
                                                robot_if.motor_drivers[i / 2].motor1->set_kp(0);
                                                robot_if.motor_drivers[i / 2].motor1->set_kd(0);
                                                }
			                                if(i==1){
                                                robot_if.motor_drivers[i / 2].motor2->set_kp(0);
                                                robot_if.motor_drivers[i / 2].motor2->set_kd(0);
                                                }


					}
				}
				 break;
		

                case 11:  //virtual spring
                                //closed loop, position
                                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                {
                                        if (i % 2 == 0)
                                        {
                                                if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                                                // making sure that the transaction with the corresponding µdriver board succeeded
                                                if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                {
                                                        //printf("Transaction with SPI%d failed\n", i / 2);
                                                        continue; //user should decide what to do in that case, here we ignore that motor
                                                }
                                        }

                                        if (robot_if.motors[i].IsEnabled())
                                        {
 
                                                if(i==0){
							                        double kp_case11_0=K_case11*fabs(initpos0_vs-robot_if.motors[i].GetPosition())+c;
                                                    robot_if.motor_drivers[i / 2].motor1->set_kp(kp_case11_0);
                                                    robot_if.motor_drivers[i / 2].motor1->set_kd(0);
                                                    robot_if.motors[i].SetPositionReference(initpos0_vs);
                                                }
                                            if(i==1){
                                                double kp_case11_1=K_case11_1*fabs(initpos1_vs-robot_if.motors[i].GetPosition())+c;
                                                robot_if.motor_drivers[i / 2].motor2->set_kp(kp_case11_1);
                                                robot_if.motor_drivers[i / 2].motor2->set_kd(0);
                                                robot_if.motors[i].SetPositionReference(initpos1_vs);
                                            }

					
                                        if(t11>duration_case11*dt && fabs(initpos1_vs-robot_if.motors[1].GetPosition())<1 && fabs(initpos0_vs-robot_if.motors[0].GetPosition())<1){ //stop virtual spring and go back to restposition
                                                robot_if.motor_drivers[i / 2].motor2->set_kp(kp);
                                                robot_if.motor_drivers[i / 2].motor2->set_kd(kd);
                                                lastpos1_case11=robot_if.motors[i].GetPosition();
                                                state= 12;
                                            }

                                        }
                                } t11+=dt;
                                 break;

                        case 12:  // arm goes up from last position with motor 0 off
                                //closed loop, position
                                for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                {
                                        if (i % 2 == 0)
                                        {
                                                if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected slave

                                                // making sure that the transaction with the corresponding µdriver board succeeded
                                                if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                {
                                                        //printf("Transaction with SPI%d failed\n", i / 2);
                                                        continue; //user should decide what to do in that case, here we ignore that motor
                                                }
                                        }

                                        if (robot_if.motors[i].IsEnabled())
                                        {
                                                // i=0 is right motor on driver board, i=1 is left motor in driver board!
                                                if(i==0){
                                                    robot_if.motor_drivers[i / 2].motor1->set_kp(0);
	                                                robot_if.motor_drivers[i / 2].motor1->set_kd(0);
                                                }

                                                if(i==1){
                                                        if(robot_if.motors[i].GetPosition()<lastpos1_case1){
                                                            state=13;
								                            lastpos0_case12=robot_if.motors[0].GetPosition();                                                        
							                            }
							                            robot_if.motor_drivers[i / 2].motor2->set_kp(kp);
                                                        robot_if.motor_drivers[i / 2].motor2->set_kd(kd);
                                                        double ref=lastpos1_case11-step_case12*t12;
                                                        robot_if.motors[i].SetPositionReference(ref);
                                                }
                                        }
                                }
                                t12+=dt;
                                break;

                        case 13: //arm  retract fully
                                 //closed loop, position
                                 for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                                 {
                                         if (i % 2 == 0)
                                         {
                                                 if (!robot_if.motor_drivers[i / 2].is_connected) continue; // ignoring the motors of a disconnected sla
                                                 // making sure that the transaction with the corresponding µdriver board succeeded
                                                 if (robot_if.motor_drivers[i / 2].error_code == 0xf)
                                                 {
                                                         //printf("Transaction with SPI%d failed\n", i / 2);
                                                         continue; //user should decide what to do in that case, here we ignore that motor
                                                 }
                                         }

                                         if (robot_if.motors[i].IsEnabled())
                                         {
                                                 if(i==1){
                                                        robot_if.motors[i].SetPositionReference(lastpos1_case1);
                                                }
                                                 if(i==0){
                                                        if(robot_if.motors[i].GetPosition()<init_pos[i]){
                                                                state=8;
                                                        }
                                                        robot_if.motor_drivers[i / 2].motor1->set_kp(kp);
                                                        robot_if.motor_drivers[i / 2].motor1->set_kd(kd);
                                                        double ref=lastpos0_case12-step_case13*t13;
                                                        robot_if.motors[i].SetPositionReference(ref);
                                                 }
                                         }
                                 }
                                t13+=dt;
                                 break;


		}
		if (cpt % 100 == 0)
			{
				printf("\33[H\33[2J"); //clear screen
				robot_if.PrintIMU();
				robot_if.PrintADC();
				robot_if.PrintMotors();
				robot_if.PrintMotorDrivers();
				robot_if.PrintStats();

				fflush(stdout);
				 

			}
			robot_if.SendCommand(); //This will send the command packet
		}
		else
		{
			std::this_thread::yield();
		}
	}
        printf("Masterboard timeout detected. Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.\n");
	return 0;
}


