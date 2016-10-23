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


using namespace iRobot;
using namespace LibSerial;
using namespace std;

#define WALL_SENSOR_MIN 4

#define MID_BACKUP_TIME_SLOT 75
#define SHORT_BACKUP_TIME_SLOT 50

#define ESCAPE_BACKUP_TIME_SLOT 75
#define ESCAPE_ROTATION_TIME_SLOT 50

#define FRONT_WALL_SEARCH_BACKUP_TIME_SLOT 100
#define FRONT_WALL_SEARCH_ROTATION_TIME_SLOT 50

#define RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT 400
#define RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT 200


#define SEARCHING_SPEED 50
#define FOLLOW_WALL_SPEED 50
#define SURVEY_SPEED 50
#define ALIGNMENT_SPEED 50

#define ALIGNMENT_THRESHOLD 0.7
#define OUTOF_CONTROL_THRESHOLD 0.4

class WallSignalManager
{
private:
    static const int WALL_SIGNAL_HISTORY_SIZE = 4;
public:
    short wallSignalHistoryArray[WALL_SIGNAL_HISTORY_SIZE];

    WallSignalManager()
    {
        for(int i=0 ; i<WALL_SIGNAL_HISTORY_SIZE ; i++)
            wallSignalHistoryArray[i] = 0;
    }

    void appendHistory(short currentWallSignal)
    {
        int index = 1;
        for(; index< WALL_SIGNAL_HISTORY_SIZE; index++)
        {
            wallSignalHistoryArray[index-1] = wallSignalHistoryArray[index];
        }
        wallSignalHistoryArray[index-1] = currentWallSignal;
    }

    short getAverage()
    {
        double sum = 0.0;

        for(int index = 0; index< WALL_SIGNAL_HISTORY_SIZE; index++)
        {
            sum += wallSignalHistoryArray[index];
        }

        return (short)(sum / (double)WALL_SIGNAL_HISTORY_SIZE);
    }

    bool isIncreasing()
    {
        int del = wallSignalHistoryArray[WALL_SIGNAL_HISTORY_SIZE-1] - wallSignalHistoryArray[0];

        return (0<=del);
    }

    bool isNoWallSignal()
    {
        return (getAverage() < WALL_SENSOR_MIN);
    }
};

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
    NS_ESCAPE_CORNER,
    NS_FOLLOW_WALL,
    NS_SEARCH_FRONT_WALL,
    NS_SEARCH_RIGHT_WALL
};

NAVIGATION_STATUS g_navigationStatus;
int g_consecutiveOperation = 0;
int g_backupTimeSlot = 0;
int g_rotationTimeSlot = 0;
bool g_NS_SURVEY_ISwallAvgHighValueSeen = false;


SurveyManager *g_surveyManagerPtr = NULL;
WallSignalManager g_wallSigMgr;
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
        //ws_resetHistory();

        while (!robot.playButton ())
        {
            short wallSignal = robot.wallSignal();
            g_wallSigMgr.appendHistory(wallSignal);

            if(false)
            {// check sensors... drop sensor etc
                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
            }
            else
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
                            if(g_wallSigMgr.isNoWallSignal())
                            {
                                g_navigationStatus = NS_SURVEY;
                                g_NS_SURVEY_ISwallAvgHighValueSeen = false;
                                g_backupTimeSlot = MID_BACKUP_TIME_SLOT;
                            }

                            else
                            {
                                g_navigationStatus = NS_ESCAPE_CORNER;
                                g_backupTimeSlot = ESCAPE_BACKUP_TIME_SLOT;
                            }

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

                        }
                        else if(robot.bumpRight())
                        {
                            if(g_wallSigMgr.isNoWallSignal())
                            {
                                g_backupTimeSlot = SHORT_BACKUP_TIME_SLOT;

                                g_navigationStatus = NS_SURVEY;
                                g_NS_SURVEY_ISwallAvgHighValueSeen = false;

                            }
                            else
                            {
                                g_backupTimeSlot = MID_BACKUP_TIME_SLOT;
                                g_navigationStatus = NS_PRE_SURVEY;
                            }

                            robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);

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
                                g_rotationTimeSlot = ESCAPE_ROTATION_TIME_SLOT;
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
                            robot.sendDriveCommand (-MID_BACKUP_TIME_SLOT, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;
                        }
                        else
                        {
                            robot.sendDriveCommand(SURVEY_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);

                            if(g_wallSigMgr.getAverage()< WALL_SENSOR_MIN)
                            {
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
                            robot.sendDriveCommand (-SURVEY_SPEED, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;
                        }
                        else
                        {

                            if(NULL == g_surveyManagerPtr)
                                g_surveyManagerPtr = new SurveyManager;

                            g_surveyManagerPtr->pushSurveyData(wallSignal);

                            if ((!g_NS_SURVEY_ISwallAvgHighValueSeen)&&(!g_wallSigMgr.isNoWallSignal()))
                                g_NS_SURVEY_ISwallAvgHighValueSeen = true;


                            if( g_NS_SURVEY_ISwallAvgHighValueSeen && (g_wallSigMgr.getAverage() < WALL_SENSOR_MIN) )
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
                            g_consecutiveOperation = 0;
                            g_backupTimeSlot = 0;

                        }
                        break;
                    }
                    case NS_FOLLOW_WALL:
                    {// REQUIREMENT: g_consecutiveOperation = 0

                        if(0 < g_backupTimeSlot)
                        {
                            robot.sendDriveCommand (-ALIGNMENT_SPEED, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;
                        }
                        else
                        {

                            if(robot.bumpLeft())
                            {
                                g_navigationStatus = NS_SEARCH_FRONT_WALL;
                                g_backupTimeSlot = FRONT_WALL_SEARCH_BACKUP_TIME_SLOT;
                            }
                            else if(robot.bumpRight())
                            {

                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                g_backupTimeSlot = SHORT_BACKUP_TIME_SLOT;
                            }
                            else
                            {
                                if(g_wallSigMgr.isNoWallSignal())
                                {
                                    g_navigationStatus = NS_SEARCH_RIGHT_WALL;
                                    g_backupTimeSlot = RIGHT_WALL_SEARCH_FORWARD_TIME_SLOT;  // shegufta: instead of declearing a new variable for forwardTimeSlot, to keep thing simple, I have just used g_backupTimeSlot

                                }
                                else
                                {
                                    if(g_consecutiveOperation < 4)
                                    {
                                        robot.sendDriveCommand(FOLLOW_WALL_SPEED, Create::DRIVE_STRAIGHT);
                                        g_consecutiveOperation++;
                                    }
                                    else
                                    {
                                        g_consecutiveOperation=0;

                                        if( g_surveyManagerPtr->getSignalStrength(wallSignal) < OUTOF_CONTROL_THRESHOLD)
                                        {
                                            robot.sendDriveCommand(ALIGNMENT_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                                        }
                                        else
                                        {
                                            if (g_wallSigMgr.isIncreasing())
                                                robot.sendDriveCommand(ALIGNMENT_SPEED,Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                                            else
                                                robot.sendDriveCommand(ALIGNMENT_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);
                                        }


                                    }

                                }

                            }

                        }

                        break;
                    }
                    case NS_SEARCH_FRONT_WALL:
                    {
                        if(0 < g_backupTimeSlot)
                        {
                            robot.sendDriveCommand (-SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;

                            if(0 == g_backupTimeSlot)
                            {
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                g_rotationTimeSlot = FRONT_WALL_SEARCH_ROTATION_TIME_SLOT;
                            }
                        }
                        else if(0 < g_rotationTimeSlot)
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

                    case NS_SEARCH_RIGHT_WALL:
                    {

                        if(0 < g_backupTimeSlot) // NOTE: here backupTimeSlot is used to move forward. I have not declared another new variable !
                        {
                            robot.sendDriveCommand (SEARCHING_SPEED, Create::DRIVE_STRAIGHT);
                            g_backupTimeSlot--;

                            if(0 == g_backupTimeSlot)
                            {
                                robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
                                g_rotationTimeSlot = RIGHT_WALL_SEARCH_ROTATION_TIME_SLOT;
                            }
                        }
                        else if(0 < g_rotationTimeSlot)
                        {
                            robot.sendDriveCommand(SEARCHING_SPEED, Create::DRIVE_INPLACE_CLOCKWISE);

                            g_rotationTimeSlot--;

                            if(0 == g_rotationTimeSlot)
                            {
                                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                                g_navigationStatus = NS_SEARCHING;
                            }
                        }

                        break;
                    }

                }

            }


            cout << "Wall signal " << robot.wallSignal() << "     sleep time "<<sleepTimeMS << "    navStatus = " << g_navigationStatus<< endl;

            this_thread::sleep_for(chrono::milliseconds(sleepTimeMS));

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
