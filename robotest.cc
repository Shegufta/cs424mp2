#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <climits>
#include <list>
#include <algorithm>
//#include <raspicam/raspicam_cv.h>
//#include <opencv2/imgproc/imgproc.hpp>


//SHRT_MIN   = minimum value for a short
//SHRT_MAX   = maximum value for a short

using namespace iRobot;
using namespace LibSerial;
using namespace std;

#define WALL_SENSOR_MIN 4
#define DEFAULT_BACKUP_TIME_SLOT 75
#define DEFAULT_BACKUP_TIME_SLOT_ESCAPE 75
#define DEFAULT_ROTATION_TIME_SLOT 5

#define SEARCHING_SPEED 50
#define FOLLOW_WALL_SPEED 50
#define SURVEY_SPEED 50
#define ALIGNMENT_SPEED 50
#define ALIGNMENT_THRESHOLD 0.7



class SurveyManager
{
public:
    std::list<short> wallSensorList;
    std::list<short>::iterator listIT;
    bool isFinalized = false;

    short min;
    short max;

    void init()
    {
        wallSensorList.clear();
    }

    SurveyManager()
    {
        init();
    }

    void pushSurveyData(short wallSensor)
    {
        if(false == isFinalized)
            wallSensorList.push_back(wallSensor);
    }

    void finalizeSurvey()
    {
        wallSensorList.sort();

        min = *(std::min_element(wallSensorList.begin(), wallSensorList.end()));
        max = *(std::max_element(wallSensorList.begin(), wallSensorList.end()));

        isFinalized = true;
    }

    double getSignalStrength(short wallSensor)
    {
        if(isFinalized)
        {
            if(wallSensor < min)
                return 0.0;
            else if(max < wallSensor)
                return 1.0;
            else
            {
                double index = 0.0;

                for(listIT = wallSensorList.begin() ; listIT != wallSensorList.end()  ; ++listIT)
                {
                    if(*listIT <= wallSensor)
                        index++;
                    else
                        break;

                }

                return (index/(double)wallSensorList.size());
            }

        }

        return -1.0;

    }



};

enum NAVIGATION_STATUS
{
    NS_SEARCHING,
    NS_PRE_SURVEY,
    NS_SURVEY,
    NS_POST_SURVEY_ALIGN,
    NS_ALIGN,
    NS_ESCAPE_CORNER,
    NS_FOLLOW_WALL,
    NS_SEARCH_LEFT_WALL,
    NS_SEARCH_RIGHT_WALL
};

NAVIGATION_STATUS g_navigationStatus;
int g_backupTimeSlot = 0;
int g_rotationTimeSlot = 0;
bool g_NS_SURVEY_ISwallAvgHighValueSeen = false;
bool g_NS_ALIGN_IsRotateClockWise = true;
SurveyManager *g_surveyManagerPtr = NULL;
///////////////////////////////////////////////////////////////////////////////
short g_wallSig_last1 = 0;
short g_wallSig_last2 = 0;
short g_wallSig_last3 = 0;

void ws_resetHistory()
{
    g_wallSig_last1 = g_wallSig_last2 = g_wallSig_last3 = 0;
}

void ws_appendHistory(short currentWallSignal)
{
    g_wallSig_last3 = g_wallSig_last2;
    g_wallSig_last2 = g_wallSig_last1;
    g_wallSig_last1 = currentWallSignal;
}

int ws_getAverage(short currentWallSignal)
{
    double sum = 0.0;
    sum = currentWallSignal + g_wallSig_last1 + g_wallSig_last2 + g_wallSig_last3;
    return (int)sum/4.0;
}
///////////////////////////////////////////////////////////////////////////////

int main ()
{
    g_navigationStatus = NS_SEARCHING;
    g_surveyManagerPtr = NULL;

    char serial_loc[] = "/dev/ttyUSB0";

    try
    {
        SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
        cout << "Opened Serial Stream to" << serial_loc << endl;
        this_thread::sleep_for(chrono::milliseconds(1000));
        Create robot(stream);
        cout << "Created iRobot Object" << endl;
        robot.sendFullCommand();
        cout << "Setting iRobot to Full Mode" << endl;
        this_thread::sleep_for(chrono::milliseconds(1000));
        cout << "Robot is ready" << endl;

        // Let's stream some sensors.
        Create::sensorPackets_t sensors;
        sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
        sensors.push_back(Create::SENSOR_WALL_SIGNAL);
        sensors.push_back (Create::SENSOR_BUTTONS);

        robot.sendStreamCommand (sensors);
        cout << "Sent Stream Command" << endl;
        // Let's turn!
        int speed = 50;
        int ledColor = Create::LED_COLOR_GREEN;
        //robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
        //robot.sendLedCommand (Create::LED_PLAY, 0, 0);
        //cout << "Sent Drive Command" << endl;



        int sleepTimeMS = 15;

        short wallSignal, prevWallSignal = 0;
        ws_resetHistory();

        while (!robot.playButton ())
        {
            short wallSignal = robot.wallSignal();

            if(false)
            {// check sensors... drop sensor etc
                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
            } else
            {
                switch(g_navigationStatus)
                {
                    case NS_SEARCHING:
                    {
                        if(0 < g_backupTimeSlot)
                        {
                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;

                            break;
                        }

                        if(NULL != g_surveyManagerPtr)
                        {
                            delete g_surveyManagerPtr;
                            g_surveyManagerPtr = NULL;
                        }

                        if(robot.bumpLeft() )
                        {
                            if(ws_getAverage(wallSignal) < WALL_SENSOR_MIN)
                            {
                                g_navigationStatus = NS_SURVEY;
                                g_NS_SURVEY_ISwallAvgHighValueSeen = false;
                                g_backupTimeSlot = DEFAULT_BACKUP_TIME_SLOT;
                            }

                            else
                            {
                                g_navigationStatus = NS_ESCAPE_CORNER;
                                g_backupTimeSlot = DEFAULT_BACKUP_TIME_SLOT_ESCAPE;

                            }

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                        }
                        else if(robot.bumpRight())
                        {
                            if(ws_getAverage(wallSignal) < WALL_SENSOR_MIN)
                            {
                                g_navigationStatus = NS_SURVEY;
                                g_NS_SURVEY_ISwallAvgHighValueSeen = false;
                            }
                            else
                                g_navigationStatus = NS_PRE_SURVEY;

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot = DEFAULT_BACKUP_TIME_SLOT;
                        }
                        else
                        {
                            robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                        }

                        break;
                    }

                    case NS_ESCAPE_CORNER:
                    {
                        if(0 < g_backupTimeSlot)
                        {
                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;

                            if(0 == g_backupTimeSlot)
                            {
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                g_rotationTimeSlot = DEFAULT_ROTATION_TIME_SLOT;
                            }
                        }else if(0 < g_rotationTimeSlot)
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                            g_rotationTimeSlot--;

                            if(0 == g_rotationTimeSlot)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                g_navigationStatus = NS_SEARCHING;
                            }
                        }
                        break;
                    }

                    case NS_PRE_SURVEY:
                    {
                        if(0 < g_backupTimeSlot)
                        {
                            robot.sendDriveCommand (-DEFAULT_BACKUP_TIME_SLOT, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;
                        }
                        else
                        {
                            robot.sendDriveCommand(SURVEY_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);

                            if(ws_getAverage(wallSignal) < WALL_SENSOR_MIN) {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                g_navigationStatus = NS_SURVEY;
                                g_NS_SURVEY_ISwallAvgHighValueSeen = false;
                            }
                        }

                        break;
                    }

                    case NS_SURVEY:
                    {// Condition: when it will enter the NS_SURVEY mode FOR THE FIRST TIME, the condition  [ws_getAverage(wallSignal) < WALL_SENSOR_MIN)] will be true
                        //CONDITION FOR THE FIRST TIME : g_NS_SURVEY_ISwallAvgHighValueSeen = false;
                        if(0 < g_backupTimeSlot)
                        {
                            robot.sendDriveCommand (-DEFAULT_BACKUP_TIME_SLOT, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;
                        }
                        else
                        {

                            if(NULL == g_surveyManagerPtr)
                                g_surveyManagerPtr = new SurveyManager;

                            g_surveyManagerPtr->pushSurveyData(wallSignal);

                            if(WALL_SENSOR_MIN <= ws_getAverage(wallSignal))
                                g_NS_SURVEY_ISwallAvgHighValueSeen = true;


                            if( g_NS_SURVEY_ISwallAvgHighValueSeen && (ws_getAverage(wallSignal) < WALL_SENSOR_MIN) )
                            {
                                g_NS_SURVEY_ISwallAvgHighValueSeen = false;
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                                g_surveyManagerPtr->finalizeSurvey();
                                g_navigationStatus = NS_POST_SURVEY_ALIGN;
                            }
                            else
                            {
                                robot.sendDriveCommand(SURVEY_SPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                            }
                        }

                        break;
                    }
                    case NS_POST_SURVEY_ALIGN:
                    {

                        if (g_surveyManagerPtr->getSignalStrength(wallSignal) < ALIGNMENT_THRESHOLD)
                            robot.sendDriveCommand(ALIGNMENT_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                        else
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                            g_navigationStatus = NS_FOLLOW_WALL;

                        }
                        break;
                    }
                    case NS_FOLLOW_WALL:
                    {
                        if(robot.bumpLeft() || robot.bumpRight() )
                        {
                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT); // TODO change this logic
                            g_backupTimeSlot = DEFAULT_BACKUP_TIME_SLOT_ESCAPE;
                            g_navigationStatus = NS_SEARCHING; // TODO  change this logic

                            //TODO flush the bump sensor
                            //backoffress
                        }
                        else
                        {
                            if (g_surveyManagerPtr->getSignalStrength(wallSignal) < ALIGNMENT_THRESHOLD)
                            {
                                robot.sendDriveCommand(ALIGNMENT_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                            }
                            else
                            {
                                robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                            }
                        }

                        break;
                    }
                    /*

                    case NS_ALIGN:
                    {
                        if(robot.bumpLeft() || robot.bumpRight() )
                        {
                            g_navigationStatus = NS_SEARCHING; // TODO  change this logic
                        }


                        break;
                    }
                    */


                }

            }




            ws_appendHistory(wallSignal);




            /*
            robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

            if(robot.bumpLeft())
            {
                if( 10 <= (sleepTimeMS - 10 ) )
                    sleepTimeMS -= 10;
            }

            if(robot.bumpRight())
            {
                if((sleepTimeMS + 10)<2000)
                    sleepTimeMS += 10;
            }
            */

            cout << "Wall signal " << robot.wallSignal() << "     sleep time "<<sleepTimeMS << "    navStatus = " << g_navigationStatus<< endl;

            this_thread::sleep_for(chrono::milliseconds(sleepTimeMS));

            /*
          if (robot.bumpLeft () || robot.bumpRight ()) {
            cout << "Bump !" << endl;
            robot.sendDriveCommand(-speed, Create::DRIVE_STRAIGHT);
            this_thread::sleep_for(chrono::milliseconds(1000));
            robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
            this_thread::sleep_for(chrono::milliseconds(300));
            robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);

          }
          short wallSignal = robot.wallSignal();
            cout << "Wall signal " << robot.wallSignal() << endl;
    */


            /*
          if (wallSignal > 0) {
            cout << "Wall signal " << robot.wallSignal() << endl;

            if (prevWallSignal == 0) {

              Camera.grab();
              Camera.retrieve (bgr_image);
              cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
              cv::imwrite("irobot_image.jpg", rgb_image);
              cout << "Taking photo" << endl;

            }
          }
            */
            //prevWallSignal = wallSignal;
            /*
          if (robot.advanceButton ())
          {
            cout << "Advance button pressed" << endl;
            speed = -1 * speed;
            ledColor += 10;
            if (ledColor > 255)
              ledColor = 0;

            robot.sendDriveCommand (speed, Create::DRIVE_INPLACE_CLOCKWISE);
            if (speed < 0)
              robot.sendLedCommand (Create::LED_PLAY,
                  ledColor,
                  Create::LED_INTENSITY_FULL);
            else
              robot.sendLedCommand (Create::LED_ADVANCE,
                  ledColor,
                  Create::LED_INTENSITY_FULL);
          }
    */
            // You can add more commands here.
            //this_thread::sleep_for(chrono::milliseconds(100));
        }

        cout << "Play button pressed, stopping Robot" << endl;
        robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
    }
    catch (InvalidArgument& e)
    {
        cerr << e.what () << endl;
        return 3;
    }
    catch (CommandNotAvailable& e)
    {
        cerr << e.what () << endl;
        return 4;
    }
}
